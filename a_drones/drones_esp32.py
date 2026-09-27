"""
Práctica Real-to-Sim — Inciso a)
Mover un enjambre de drones del punto A al punto B y al punto C,
con el control gestionado desde una ESP32 (botones -> UART/USB -> PC -> PyBullet).

Basado en gym-pybullet-drones (CtrlAviary + DSLPIDControl).

USO (desde la carpeta raíz del repositorio gym-pybullet-drones):
    python drones_esp32.py --port COM5          # con la ESP32 (Windows)
    python drones_esp32.py --port /dev/ttyUSB0  # con la ESP32 (Linux)
    python drones_esp32.py                      # sin ESP32: teclado 1/2/3/4

Protocolo serie (una línea por comando, 115200 baudios):
    CMD:A   -> ir al punto A
    CMD:B   -> ir al punto B
    CMD:C   -> ir al punto C
    CMD:S   -> secuencia automática A -> B -> C
    CMD:L   -> aterrizar en el sitio actual
"""
import time
import queue
import argparse
import threading

import numpy as np
import pybullet as p

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool

# ---------------------------------------------------------------------------
# Puntos de destino (centro de la formación) [x, y, z] en metros.
# Elegidos para no chocar con la esfera (0, 2, 0.5) ni con el cubo (-0.5,-2.5).
# ---------------------------------------------------------------------------
PUNTOS = {
    "A": np.array([0.0, 0.0, 0.6]),
    "B": np.array([1.2, 1.0, 1.0]),
    "C": np.array([-1.2, 0.8, 0.8]),
}
COLORES = {"A": [1, 0, 0], "B": [0, 0.6, 0], "C": [0, 0, 1]}

VEL_MEDIA = 0.4        # m/s, velocidad media del traslado de la formación
T_MIN = 2.0            # s, duración mínima de un traslado
TOL_LLEGADA = 0.05     # m, error máximo para considerar que llegó
ALT_SUELO = 0.08       # m, altura de "aterrizado"


# ---------------------------------------------------------------------------
# Trayectoria de mínimo jerk (suave: velocidad y aceleración nulas en extremos)
# ---------------------------------------------------------------------------
class TrayectoriaMinJerk:
    def __init__(self, p0, pf, t0, duracion):
        self.p0, self.pf = np.array(p0, float), np.array(pf, float)
        self.t0, self.T = t0, max(duracion, 1e-3)

    def evaluar(self, t):
        s = np.clip((t - self.t0) / self.T, 0.0, 1.0)
        pos_s = 10 * s**3 - 15 * s**4 + 6 * s**5
        vel_s = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.T
        d = self.pf - self.p0
        return self.p0 + pos_s * d, vel_s * d

    def terminada(self, t):
        return t >= self.t0 + self.T


# ---------------------------------------------------------------------------
# Lectura del puerto serie en un hilo aparte (no bloquea la simulación)
# ---------------------------------------------------------------------------
def hilo_serial(puerto, baudios, cola, detener):
    import serial  # pyserial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = puerto, baudios, 0.1
    ser.dtr, ser.rts = False, False       # evita que la ESP32 se reinicie al abrir
    try:
        ser.open()
    except serial.SerialException as e:
        print(f"[SERIAL] No se pudo abrir {puerto}: {e}")
        print("[SERIAL] ¿Está Thonny/Arduino IDE usando el puerto? Ciérralo.")
        cola.put("__ERROR__")
        return
    print(f"[SERIAL] Conectado a {puerto} @ {baudios}")
    buffer = b""
    while not detener.is_set():
        try:
            buffer += ser.read(64)
        except serial.SerialException:
            print("[SERIAL] Se perdió la conexión con la ESP32.")
            break
        while b"\n" in buffer:
            linea, buffer = buffer.split(b"\n", 1)
            texto = linea.decode(errors="ignore").strip()
            if texto.startswith("CMD:"):
                cola.put(texto[4:].upper())
            elif texto:
                print(f"[ESP32] {texto}")
    ser.close()


def run(args):
    N = args.num_drones
    ctrl_freq = args.control_freq_hz

    # ---- Formación en rejilla alrededor del centro ------------------------
    cols = int(np.ceil(np.sqrt(N)))
    OFFSETS = np.array([[(i % cols - (cols - 1) / 2) * args.separacion,
                         (i // cols - (int(np.ceil(N / cols)) - 1) / 2) * args.separacion,
                         0.0] for i in range(N)])

    centro_ini = PUNTOS["A"].copy()
    centro_ini[2] = ALT_SUELO
    INIT_XYZS = centro_ini + OFFSETS
    INIT_RPYS = np.zeros((N, 3))

    env = CtrlAviary(drone_model=DroneModel.CF2X,
                     num_drones=N,
                     initial_xyzs=INIT_XYZS,
                     initial_rpys=INIT_RPYS,
                     physics=Physics.PYB,
                     neighbourhood_radius=10,
                     pyb_freq=args.simulation_freq_hz,
                     ctrl_freq=ctrl_freq,
                     gui=args.gui,
                     obstacles=True,          # samurái, pato, cubo y esfera (como en la guía)
                     user_debug_gui=False)
    CLIENTE = env.getPyBulletClient()
    ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(N)]

    # ---- Marcadores visuales de A, B y C ----------------------------------
    if args.gui:
        p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=-35, cameraPitch=-35,
                                     cameraTargetPosition=[0, 0.5, 0.3],
                                     physicsClientId=CLIENTE)
        for nombre, pt in PUNTOS.items():
            p.addUserDebugLine([pt[0], pt[1], 0], pt.tolist(), COLORES[nombre], 2,
                               physicsClientId=CLIENTE)
            p.addUserDebugText(f"Punto {nombre}", (pt + [0, 0, 0.15]).tolist(),
                               COLORES[nombre], 1.5, physicsClientId=CLIENTE)

    # ---- Fuente de comandos ------------------------------------------------
    cola = queue.Queue()
    detener = threading.Event()
    if args.port:
        threading.Thread(target=hilo_serial, args=(args.port, args.baud, cola, detener),
                         daemon=True).start()
    else:
        print("[INFO] Sin --port: usa el teclado en la ventana de PyBullet")
    print("[INFO] Teclas: 1=A  2=B  3=C  4=Secuencia A->B->C  5=Aterrizar")
    TECLAS = {ord("1"): "A", ord("2"): "B", ord("3"): "C", ord("4"): "S", ord("5"): "L"}

    # ---- Estado del "planificador" ----------------------------------------
    t_sim = 0.0
    centro = centro_ini.copy()
    tray = TrayectoriaMinJerk(centro, centro, 0.0, 0.0)
    pendientes = []            # cola de destinos (para la secuencia)
    destino_actual = None
    reportado = True

    def ir_a(nombre_o_pos, etiqueta):
        nonlocal tray, destino_actual, reportado
        pf = nombre_o_pos
        dist = np.linalg.norm(pf - centro)
        tray = TrayectoriaMinJerk(centro, pf, t_sim, max(T_MIN, dist / VEL_MEDIA))
        destino_actual, reportado = etiqueta, False
        print(f"[t={t_sim:6.2f}s] -> Rumbo a {etiqueta} {np.round(pf, 2)} "
              f"(dist {dist:.2f} m, {tray.T:.1f} s)")

    # Despegue automático hasta el punto A
    ir_a(PUNTOS["A"], "A (despegue)")

    action = np.zeros((N, 4))
    obs, *_ = env.step(action)
    pasos = int(args.duration_sec * ctrl_freq) if args.duration_sec > 0 else None
    INICIO = time.time()
    i = 0
    llegadas = []  # (tiempo, destino, error) para el reporte
    try:
        while pasos is None or i < pasos:
            # 1) Leer comandos (ESP32 y/o teclado)
            nuevos = []
            while not cola.empty():
                nuevos.append(cola.get())
            if args.gui:
                for k, estado in p.getKeyboardEvents(physicsClientId=CLIENTE).items():
                    if k in TECLAS and estado & p.KEY_WAS_TRIGGERED:
                        nuevos.append(TECLAS[k])
            for cmd in nuevos:
                if cmd == "__ERROR__":
                    raise SystemExit(1)
                print(f"[CMD] {cmd}")
                if cmd in PUNTOS:
                    pendientes = []
                    ir_a(PUNTOS[cmd], cmd)
                elif cmd == "S":
                    pendientes = ["B", "C"]
                    ir_a(PUNTOS["A"], "A")
                elif cmd == "L":
                    pendientes = []
                    ir_a(np.array([centro[0], centro[1], ALT_SUELO]), "suelo")

            # 2) Avanzar la trayectoria de la formación
            centro, vel = tray.evaluar(t_sim)

            # 3) Control PID de cada dron hacia su posición en la formación
            for j in range(N):
                action[j, :], _, _ = ctrl[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[j],
                    target_pos=centro + OFFSETS[j],
                    target_rpy=INIT_RPYS[j],
                    target_vel=vel)
            obs, *_ = env.step(action)
            t_sim += env.CTRL_TIMESTEP

            # 4) ¿Llegó la formación?
            if not reportado and tray.terminada(t_sim):
                pos = np.array([obs[j][0:3] for j in range(N)])
                err = np.linalg.norm(pos - (tray.pf + OFFSETS), axis=1).max()
                if err < TOL_LLEGADA:
                    print(f"[t={t_sim:6.2f}s] OK: formación en {destino_actual} "
                          f"(error máx {100*err:.1f} cm)")
                    llegadas.append((round(t_sim, 2), destino_actual, err))
                    reportado = True
                    if pendientes:
                        sig = pendientes.pop(0)
                        ir_a(PUNTOS[sig], sig)

            if args.gui:
                sync(i, INICIO, env.CTRL_TIMESTEP)
            i += 1
    except KeyboardInterrupt:
        pass
    except p.error:
        print("[INFO] Ventana de PyBullet cerrada.")
    finally:
        detener.set()
        env.close()
    return llegadas


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Drones A->B->C controlados por ESP32")
    ap.add_argument("--port", default=None, help="Puerto de la ESP32 (ej. COM5). Vacío = teclado")
    ap.add_argument("--baud", default=115200, type=int)
    ap.add_argument("--num_drones", default=9, type=int)
    ap.add_argument("--separacion", default=0.3, type=float, help="Distancia entre drones (m)")
    ap.add_argument("--gui", default=True, type=str2bool)
    ap.add_argument("--simulation_freq_hz", default=240, type=int)
    ap.add_argument("--control_freq_hz", default=48, type=int)
    ap.add_argument("--duration_sec", default=0, type=float, help="0 = hasta cerrar la ventana")
    run(ap.parse_args())
