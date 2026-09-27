"""
Práctica Real-to-Sim — Inciso b)
Consola de mandos con ESP32 (3 potenciómetros + 5 pulsadores) para mover los
brazos del robot Baxter con cinemática inversa, coger un cubo y moverlo.

Basado en baxter_ik_demo.py del repositorio erwincoumans/pybullet_robots.

USO (desde la carpeta raíz del repositorio pybullet_robots):
    python baxter_esp32.py --port COM5     # con la ESP32
    python baxter_esp32.py                 # sin ESP32: sliders + teclado

Protocolo serie (115200 baudios, una línea por mensaje):
    POT:x,y,z        lecturas de los 3 potenciómetros (0..4095)
    BTN:PINZA        abrir / cerrar la pinza
    BTN:BRAZO        cambiar entre brazo izquierdo y derecho
    BTN:HOME         llevar el brazo activo a su posición de reposo
    BTN:RESET        devolver los cubos a su lugar
    BTN:MUNECA       el potenciómetro 3 pasa a girar la muñeca (y viceversa)
"""
import os
import time
import queue
import argparse
import threading

import numpy as np
import pybullet as p
import pybullet_data

DIR = os.path.dirname(os.path.abspath(__file__))
URDF_BAXTER = os.path.join(DIR, "data", "baxter_common", "baxter_description",
                           "urdf", "toms_baxter.urdf")

# ---------------------------------------------------------------------------
# Escena
# ---------------------------------------------------------------------------
SUELO = -0.93            # el origen del Baxter está a la altura del torso
MESA_Z = -0.15           # altura de la superficie de la mesa
MESA_X = (0.45, 1.00)    # la mesa va de x=0.45 a x=1.00 frente al robot
MESA_Y = 0.60            # y la mitad del ancho
LADO_CUBO = 0.04         # 4 cm: cabe entre los dedos abiertos (≈5.5 cm)
CUBOS_INI = {            # posición inicial y color de cada cubo
    "verde": ([0.65, 0.30], [0.1, 0.7, 0.1, 1]),
    "azul":  ([0.65, -0.30], [0.1, 0.3, 0.9, 1]),
}
ZONA_ENTREGA = [0.65, 0.0]  # cuadro rojo en el centro de la mesa

# ---------------------------------------------------------------------------
# Brazos del Baxter (índices de articulaciones del URDF toms_baxter.urdf)
# ---------------------------------------------------------------------------
BRAZOS = {
    "izquierdo": dict(efector=48, juntas=[34, 35, 36, 37, 38, 40, 41],
                      dedos=(49, 51), signo_y=+1),
    "derecho":   dict(efector=26, juntas=[12, 13, 14, 15, 16, 18, 19],
                      dedos=(27, 29), signo_y=-1),
}
# Postura de reposo ("untuck") — también sirve de postura preferida para la IK
POSTURA_REPOSO = {34: 0.08, 35: -1.0, 36: -1.19, 37: 1.94, 38: 0.67, 40: 1.03, 41: -0.5,
                  12: -0.08, 13: -1.0, 14: 1.19, 15: 1.94, 16: -0.67, 18: 1.03, 19: 0.5}
DEDO_ABIERTO = 0.02      # recorrido de cada dedo (m)
FUERZA_DEDO = 20.0       # N (máximo del URDF)

# Espacio de trabajo alcanzable con la pinza apuntando hacia abajo (medido)
X_RANGO = (0.50, 0.80)
Y_RANGO_IZQ = (-0.10, 0.55)     # el derecho es el espejo: (-0.55, 0.10)
Z_RANGO = (MESA_Z + 0.015, MESA_Z + 0.35)
YAW_RANGO = (-np.pi / 2, np.pi / 2)

VEL_MAX = 0.30           # m/s   — velocidad máxima del objetivo (movimiento fluido)
VEL_YAW = 1.5            # rad/s — velocidad máxima de giro de muñeca
ALFA_FILTRO = 0.15       # filtro pasa-bajos sobre la lectura de los potenciómetros
UMBRAL_TOMA = 150        # cuentas ADC que hay que mover un pot para "retomar" el control
HZ_SIM = 240
HZ_IK = 60


def mapear(v, rango):
    return rango[0] + (rango[1] - rango[0]) * np.clip(v / 4095.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Lectura del puerto serie en un hilo aparte
# ---------------------------------------------------------------------------
class ConsolaESP32(threading.Thread):
    def __init__(self, puerto, baudios):
        super().__init__(daemon=True)
        self.puerto, self.baudios = puerto, baudios
        self.pots = None                # última lectura [p1, p2, p3]
        self.eventos = queue.Queue()
        self.error = None
        self._stop = threading.Event()

    def run(self):
        import serial
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = self.puerto, self.baudios, 0.05
        ser.dtr, ser.rts = False, False          # no reiniciar la ESP32 al abrir
        try:
            ser.open()
        except serial.SerialException as e:
            self.error = f"No se pudo abrir {self.puerto}: {e}. ¿Thonny sigue abierto?"
            return
        print(f"[SERIAL] Conectado a {self.puerto} @ {self.baudios}")
        buf = b""
        while not self._stop.is_set():
            try:
                buf += ser.read(256)
            except serial.SerialException:
                self.error = "Se perdió la conexión con la ESP32"
                break
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                t = linea.decode(errors="ignore").strip()
                if t.startswith("POT:"):
                    try:
                        self.pots = [float(v) for v in t[4:].split(",")][:3]
                    except ValueError:
                        pass
                elif t.startswith("BTN:"):
                    self.eventos.put(t[4:].upper())
                elif t:
                    print(f"[ESP32] {t}")
        ser.close()

    def detener(self):
        self._stop.set()


# ---------------------------------------------------------------------------
# Construcción del mundo
# ---------------------------------------------------------------------------
def crear_caja(medias, pos, color, masa=0.0):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=medias)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=medias, rgbaColor=color)
    return p.createMultiBody(masa, col, vis, pos)


def crear_mundo(gui, panel=True):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / HZ_SIM)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.loadURDF("plane.urdf", [0, 0, SUELO])
    baxter = p.loadURDF(URDF_BAXTER, useFixedBase=True)

    # mesa: tablero de 4 cm + 4 patas
    cx, hx = (MESA_X[0] + MESA_X[1]) / 2, (MESA_X[1] - MESA_X[0]) / 2
    madera = [0.55, 0.38, 0.22, 1]
    crear_caja([hx, MESA_Y, 0.02], [cx, 0, MESA_Z - 0.02], madera)
    alto_pata = (MESA_Z - 0.04 - SUELO) / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            crear_caja([0.03, 0.03, alto_pata],
                       [cx + sx * (hx - 0.05), sy * (MESA_Y - 0.05), SUELO + alto_pata],
                       [0.35, 0.24, 0.14, 1])
    # zona de entrega (solo visual, sin colisión)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.001],
                              rgbaColor=[0.9, 0.1, 0.1, 1])
    p.createMultiBody(0, -1, vis, [ZONA_ENTREGA[0], ZONA_ENTREGA[1], MESA_Z + 0.001])

    cubos = {}
    h = LADO_CUBO / 2
    for nombre, (xy, color) in CUBOS_INI.items():
        cubos[nombre] = crear_caja([h, h, h], [xy[0], xy[1], MESA_Z + h], color, masa=0.1)
        p.changeDynamics(cubos[nombre], -1, lateralFriction=1.5, spinningFriction=0.01)

    # dedos con buena fricción para sujetar el cubo
    for brazo in BRAZOS.values():
        for d in brazo["dedos"]:
            p.changeDynamics(baxter, d, lateralFriction=1.5)
            p.changeDynamics(baxter, d + 1, lateralFriction=1.5)   # punta del dedo

    for j, v in POSTURA_REPOSO.items():
        p.resetJointState(baxter, j, v)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if panel else 0)
        for vista in (p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                      p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
            p.configureDebugVisualizer(vista, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        # vista frontal (como si el operador estuviera frente al robot):
        # el brazo IZQUIERDO del Baxter se ve a la DERECHA de la pantalla
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-30,
                                     cameraTargetPosition=[0.55, 0.0, 0.05])
    return baxter, cubos


def resetear_cubos(cubos):
    h = LADO_CUBO / 2
    for nombre, (xy, _) in CUBOS_INI.items():
        p.resetBasePositionAndOrientation(cubos[nombre], [xy[0], xy[1], MESA_Z + h], [0, 0, 0, 1])
        p.resetBaseVelocity(cubos[nombre], [0, 0, 0], [0, 0, 0])


# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------
def run(args):
    p.connect(p.GUI if args.gui else p.DIRECT)
    baxter, cubos = crear_mundo(args.gui, panel=not args.port)

    movibles = [i for i in range(p.getNumJoints(baxter)) if p.getJointInfo(baxter, i)[3] > -1]
    idx = {j: k for k, j in enumerate(movibles)}
    lim_inf = [p.getJointInfo(baxter, j)[8] for j in movibles]
    lim_sup = [p.getJointInfo(baxter, j)[9] for j in movibles]
    rangos = [s - i for i, s in zip(lim_inf, lim_sup)]
    reposo = [POSTURA_REPOSO.get(j, 0.0) for j in movibles]

    # estado de cada brazo: objetivo cartesiano, giro de muñeca, pinza, ángulos
    estado = {}
    for nombre, b in BRAZOS.items():
        pos = np.array(p.getLinkState(baxter, b["efector"])[4])
        estado[nombre] = dict(obj=pos.copy(), consigna=pos.copy(), yaw=0.0, yaw_c=0.0,
                              pinza_cerrada=False,
                              q=[POSTURA_REPOSO[j] for j in b["juntas"]])
    home = {n: estado[n]["obj"].copy() for n in BRAZOS}
    activo = "izquierdo"
    modo_muneca = False

    # --- entrada: ESP32 o sliders/teclado -------------------------------------
    consola = None
    sliders = None
    if args.port:
        consola = ConsolaESP32(args.port, args.baud)
        consola.start()
    else:
        print("[INFO] Sin --port: usa los sliders 'Pot' y el teclado en la ventana")
        if args.gui:
            sliders = [p.addUserDebugParameter(f"Pot{i+1}", 0, 4095, 2048) for i in range(3)]
    print("[INFO] Teclas: 1=Pinza 2=Cambiar brazo 3=Home 4=Reset cubos 5=Modo muñeca")
    TECLAS = {ord("1"): "PINZA", ord("2"): "BRAZO", ord("3"): "HOME",
              ord("4"): "RESET", ord("5"): "MUNECA"}

    pots_filt = None
    pots_toma = None          # lectura en el momento en que se "congeló" el control
    tomado = [False, False, False]

    def congelar(cuales=(0, 1, 2)):
        """Tras cambiar de brazo/modo o ir a HOME, el brazo no salta: espera a que
        el usuario mueva cada potenciómetro para retomar el control."""
        nonlocal pots_toma
        if pots_filt is None:
            return
        for i in cuales:
            pots_toma[i] = pots_filt[i]
            tomado[i] = False

    textos = [-1, -1]
    marcador = None
    if args.gui:
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=[1, 0, 0, 0.6])
        marcador = p.createMultiBody(0, -1, vis, estado[activo]["obj"].tolist())

    def hud():
        if not args.gui:
            return
        e = estado[activo]
        lado = "cubo verde" if activo == "izquierdo" else "cubo azul"
        lineas = [f"Brazo: {activo.upper()} ({lado})",
                  f"Pinza: {'CERRADA' if e['pinza_cerrada'] else 'abierta'}  |  "
                  f"Pot3: {'MUNECA' if modo_muneca else 'ALTURA'}"]
        for k, txt in enumerate(lineas):
            textos[k] = p.addUserDebugText(txt, [0.55, -1.45, 0.62 - 0.08 * k], [0, 0, 0], 1.1,
                                           replaceItemUniqueId=textos[k])

    hud()
    t0 = time.time()
    paso = 0
    pasos_ik = HZ_SIM // HZ_IK
    dt_ik = 1.0 / HZ_IK
    duracion = int(args.duration_sec * HZ_SIM) if args.duration_sec > 0 else None
    try:
        while duracion is None or paso < duracion:
            # -------- 1) eventos de botones ---------------------------------
            eventos = []
            if consola:
                if consola.error:
                    print("[SERIAL]", consola.error)
                    break
                while not consola.eventos.empty():
                    eventos.append(consola.eventos.get())
            if args.gui:
                for k, st in p.getKeyboardEvents().items():
                    if k in TECLAS and st & p.KEY_WAS_TRIGGERED:
                        eventos.append(TECLAS[k])
            for ev in eventos:
                e = estado[activo]
                if ev == "PINZA":
                    e["pinza_cerrada"] = not e["pinza_cerrada"]
                elif ev == "BRAZO":
                    activo = "derecho" if activo == "izquierdo" else "izquierdo"
                    congelar()
                elif ev == "HOME":
                    e["consigna"] = home[activo].copy()
                    e["yaw_c"] = 0.0
                    congelar()
                elif ev == "RESET":
                    resetear_cubos(cubos)
                elif ev == "MUNECA":
                    modo_muneca = not modo_muneca
                    congelar((2,))
                print(f"[BTN] {ev:6s} -> brazo {activo}, pinza "
                      f"{'cerrada' if estado[activo]['pinza_cerrada'] else 'abierta'}, "
                      f"pot3={'muñeca' if modo_muneca else 'altura'}")
                hud()

            # -------- 2) potenciómetros -> consigna (a 60 Hz) ---------------
            if paso % pasos_ik == 0:
                crudo = None
                if consola and consola.pots is not None:
                    crudo = np.array(consola.pots, float)
                elif sliders:
                    crudo = np.array([p.readUserDebugParameter(s) for s in sliders])
                if crudo is not None:
                    pots_filt = crudo if pots_filt is None else \
                        pots_filt + ALFA_FILTRO * (crudo - pots_filt)
                    if pots_toma is None:
                        pots_toma = pots_filt.copy()
                        tomado = [True, True, True]
                    for i in range(3):
                        if not tomado[i] and abs(pots_filt[i] - pots_toma[i]) > UMBRAL_TOMA:
                            tomado[i] = True
                    e = estado[activo]
                    s = BRAZOS[activo]["signo_y"]
                    y_rango = Y_RANGO_IZQ if s > 0 else (-Y_RANGO_IZQ[1], -Y_RANGO_IZQ[0])
                    if tomado[0]:
                        e["consigna"][0] = mapear(pots_filt[0], X_RANGO)
                    if tomado[1]:
                        e["consigna"][1] = mapear(pots_filt[1], y_rango)
                    if tomado[2]:
                        if modo_muneca:
                            e["yaw_c"] = mapear(pots_filt[2], YAW_RANGO)
                        else:
                            e["consigna"][2] = mapear(pots_filt[2], Z_RANGO)

                # -------- 3) objetivo suave (limitador de velocidad) --------
                for nombre, e in estado.items():
                    d = e["consigna"] - e["obj"]
                    dist = np.linalg.norm(d)
                    if dist > VEL_MAX * dt_ik:
                        d *= VEL_MAX * dt_ik / dist
                    e["obj"] = e["obj"] + d
                    e["yaw"] += np.clip(e["yaw_c"] - e["yaw"], -VEL_YAW * dt_ik, VEL_YAW * dt_ik)

                # -------- 4) cinemática inversa del brazo activo ------------
                e = estado[activo]
                b = BRAZOS[activo]
                orn = p.getQuaternionFromEuler([0, np.pi, e["yaw"]])   # pinza hacia abajo
                q = p.calculateInverseKinematics(
                    baxter, b["efector"], e["obj"].tolist(), orn,
                    lowerLimits=lim_inf, upperLimits=lim_sup, jointRanges=rangos,
                    restPoses=reposo, maxNumIterations=100, residualThreshold=1e-4)
                e["q"] = [q[idx[j]] for j in b["juntas"]]
                if marcador is not None:
                    p.resetBasePositionAndOrientation(marcador, e["obj"].tolist(), [0, 0, 0, 1])

            # -------- 5) motores: ambos brazos mantienen su consigna ---------
            for nombre, b in BRAZOS.items():
                e = estado[nombre]
                for j, qj in zip(b["juntas"], e["q"]):
                    p.setJointMotorControl2(baxter, j, p.POSITION_CONTROL, qj,
                                            force=p.getJointInfo(baxter, j)[10], maxVelocity=1.5)
                apertura = 0.0 if e["pinza_cerrada"] else DEDO_ABIERTO
                p.setJointMotorControl2(baxter, b["dedos"][0], p.POSITION_CONTROL, apertura,
                                        force=FUERZA_DEDO)
                p.setJointMotorControl2(baxter, b["dedos"][1], p.POSITION_CONTROL, -apertura,
                                        force=FUERZA_DEDO)

            p.stepSimulation()
            paso += 1
            if args.gui or consola:          # tiempo real
                atraso = paso / HZ_SIM - (time.time() - t0)
                if atraso > 0:
                    time.sleep(atraso)
    except KeyboardInterrupt:
        pass
    except p.error:
        print("[INFO] Ventana cerrada.")
    finally:
        if consola:
            consola.detener()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Consola ESP32 para el robot Baxter")
    ap.add_argument("--port", default=None, help="Puerto de la ESP32 (ej. COM5). Vacío = sliders")
    ap.add_argument("--baud", default=115200, type=int)
    ap.add_argument("--gui", default=True, type=lambda s: s.lower() in ("1", "true", "si", "yes"))
    ap.add_argument("--duration_sec", default=0, type=float, help="0 = hasta cerrar la ventana")
    run(ap.parse_args())
