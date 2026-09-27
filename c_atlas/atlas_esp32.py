"""
Práctica Real-to-Sim — Inciso c)
Consola de mandos con ESP32 (3 potenciómetros + 5 pulsadores) para mover el
robot humanoide Atlas en el laboratorio "botlab" de PyBullet.

Basado en atlas.py del repositorio erwincoumans/pybullet_robots.
Usa la MISMA consola y el MISMO main.py de la ESP32 del inciso b).

USO (desde la carpeta raíz del repositorio pybullet_robots):
    python atlas_esp32.py --port COM5     # con la ESP32
    python atlas_esp32.py                 # sin ESP32: sliders + teclado
    python atlas_esp32.py --arnes 0       # sin arnés virtual: física libre (se puede caer)

Mensajes que llegan de la ESP32 (115200 baudios):
    POT:x,y,z      3 potenciómetros (0..4095)
    BTN:PINZA      (GPIO25) -> siguiente modo de control
    BTN:BRAZO      (GPIO26) -> gesto de saludo
    BTN:MUNECA     (GPIO27) -> cámara sintética de la cabeza ON/OFF
    BTN:HOME       (GPIO33) -> postura inicial
    BTN:RESET      (GPIO32) -> levantar al robot si se cae
"""
import os
import time
import math
import queue
import argparse
import threading
import unicodedata

import numpy as np
import pybullet as p
import pybullet_data

DIR = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(DIR, "data")

POS_ATLAS = [-2, 3, -0.5]
HZ_SIM = 240
HZ_CTRL = 60
VEL_MAX = 1.2              # rad/s máx. de brazos, torso y cabeza -> movimiento fluido
VEL_MAX_PIERNAS = 0.6      # rad/s máx. de las piernas (cargan todo el peso)
ALFA_FILTRO = 0.2          # filtro pasa-bajos sobre los potenciómetros
UMBRAL_TOMA = 150          # cuentas ADC para "retomar" una perilla tras cambiar de modo

# ---------------------------------------------------------------------------
# Modos de control: cada modo asigna los 3 potenciómetros a 3 movimientos.
# Cada movimiento = (nombre visible, {articulación: (valor_pot_0, valor_pot_4095)})
# Un movimiento puede mover varias articulaciones a la vez (sentadilla, espejo...).
# ---------------------------------------------------------------------------
BRAZO_IZQ = [
    ("Hombro sube/baja", {"l_arm_shx": (-1.5, 1.2)}),
    ("Hombro adelante/atrás", {"l_arm_shz": (-1.2, 0.6)}),
    ("Codo", {"l_arm_elx": (0.0, 2.2)}),
]
BRAZO_DER = [   # espejo del izquierdo: misma perilla -> mismo gesto
    ("Hombro sube/baja", {"r_arm_shx": (1.5, -1.2)}),
    ("Hombro adelante/atrás", {"r_arm_shz": (1.2, -0.6)}),
    ("Codo", {"r_arm_elx": (0.0, -2.2)}),
]
MODOS = [
    ("BRAZO IZQUIERDO", BRAZO_IZQ),
    ("BRAZO DERECHO", BRAZO_DER),
    ("AMBOS BRAZOS (espejo)", [(n, {**a, **b}) for (n, a), (_, b) in zip(BRAZO_IZQ, BRAZO_DER)]),
    ("TORSO Y CABEZA", [
        ("Girar cintura", {"back_bkz": (-0.6, 0.6)}),
        ("Inclinar adelante", {"back_bky": (-0.2, 0.5)}),
        ("Cabeza arriba/abajo", {"neck_ry": (-0.5, 1.0)}),
    ]),
    ("PIERNAS", [
        ("Sentadilla", {"l_leg_hpy": (0.0, -0.9), "l_leg_kny": (0.0, 1.8), "l_leg_aky": (0.0, -0.9),
                        "r_leg_hpy": (0.0, -0.9), "r_leg_kny": (0.0, 1.8), "r_leg_aky": (0.0, -0.9)}),
        ("Cadera a los lados", {"l_leg_hpx": (-0.15, 0.15), "r_leg_hpx": (-0.15, 0.15),
                                "l_leg_akx": (0.15, -0.15), "r_leg_akx": (0.15, -0.15)}),
        ("Inclinar torso a los lados", {"back_bkx": (-0.4, 0.4)}),
    ]),
]


def sin_tildes(txt):
    """El texto 3D de PyBullet no dibuja tildes ni eñes."""
    return unicodedata.normalize("NFKD", txt).encode("ascii", "ignore").decode()


def mapear(v, lo, hi):
    return lo + (hi - lo) * float(np.clip(v / 4095.0, 0.0, 1.0))


# ---------------------------------------------------------------------------
# Lectura del puerto serie (igual que en el inciso b)
# ---------------------------------------------------------------------------
class ConsolaESP32(threading.Thread):
    def __init__(self, puerto, baudios):
        super().__init__(daemon=True)
        self.puerto, self.baudios = puerto, baudios
        self.pots = None
        self.eventos = queue.Queue()
        self.error = None
        self._stop = threading.Event()

    def run(self):
        import serial
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = self.puerto, self.baudios, 0.05
        ser.dtr, ser.rts = False, False
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
# Mundo (como atlas.py: laboratorio botlab + caja de Boston Dynamics)
# ---------------------------------------------------------------------------
def crear_mundo(gui, panel=True):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    atlas = p.loadURDF(os.path.join(DATA, "atlas", "atlas_v4_with_multisense.urdf"),
                       POS_ATLAS)
    objs = p.loadSDF(os.path.join(DATA, "botlab", "botlab.sdf"), globalScaling=2.0)
    y2x = p.getQuaternionFromEuler([math.pi / 2, 0, math.pi / 2])   # el SDF viene con Y hacia arriba
    for o in objs:
        pos, orn = p.getBasePositionAndOrientation(o)
        npos, norn = p.multiplyTransforms([0, 0, 0], y2x, pos, orn)
        p.resetBasePositionAndOrientation(o, npos, norn)
    p.loadURDF(os.path.join(DATA, "boston_box.urdf"), [-2, 3, -2], useFixedBase=True)
    p.loadURDF(os.path.join(DATA, "boston_box.urdf"), [0, 3, -2], useFixedBase=True)
    p.setGravity(0, 0, -10)
    p.setTimeStep(1.0 / HZ_SIM)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if panel else 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=120, cameraPitch=-12,
                                     cameraTargetPosition=[-2, 3, -0.6])
    return atlas


def main(args):
    p.connect(p.GUI if args.gui else p.DIRECT)
    atlas = crear_mundo(args.gui, panel=True)
    J = {p.getJointInfo(atlas, i)[1].decode(): i for i in range(p.getNumJoints(atlas))}
    LIM = {n: (p.getJointInfo(atlas, i)[8], p.getJointInfo(atlas, i)[9]) for n, i in J.items()}
    FUERZA = {n: p.getJointInfo(atlas, i)[10] for n, i in J.items()}

    objetivo = {n: 0.0 for n in J}      # consigna que piden las perillas
    comando = {n: 0.0 for n in J}       # consigna suavizada que recibe el motor

    # ---- arnés virtual -------------------------------------------------------
    # Los humanoides reales se prueban colgados de un arnés de seguridad. Aquí el
    # arnés es una restricción que mantiene la pelvis vertical y la mueve para que
    # los pies se queden quietos sobre la caja: así la sentadilla y el balanceo de
    # cadera se ven reales y el robot no se cae mientras aprendes a manejarlo.
    PIES = (J["l_leg_akx"], J["r_leg_akx"])
    arnes = {"id": None, "pies0": None}
    # "fantasma": copia cinemática del Atlas en un mundo aparte (sin física) para saber
    # dónde quedan los pies respecto a la pelvis con los ángulos que se están pidiendo
    fantasma_cli = p.connect(p.DIRECT)
    fantasma = p.loadURDF(os.path.join(DATA, "atlas", "atlas_v4_with_multisense.urdf"),
                          useFixedBase=True, physicsClientId=fantasma_cli)

    def pies():
        return np.mean([p.getLinkState(atlas, f)[4] for f in PIES], axis=0)

    def pies_relativos(angulos):
        for n, q in angulos.items():
            p.resetJointState(fantasma, J[n], q, physicsClientId=fantasma_cli)
        base = np.array(p.getBasePositionAndOrientation(fantasma, physicsClientId=fantasma_cli)[0])
        return np.mean([p.getLinkState(fantasma, f, computeForwardKinematics=1,
                                       physicsClientId=fantasma_cli)[4] for f in PIES], axis=0) - base

    def asentar_y_colgar():
        if arnes["id"] is not None:             # soltar el arnés mientras se apoya
            p.changeConstraint(arnes["id"], maxForce=0)
        for n, i in J.items():
            p.setJointMotorControl2(atlas, i, p.POSITION_CONTROL, 0.0, force=FUERZA[n])
        for _ in range(HZ_SIM):                 # 1 s para que los pies se apoyen en la caja
            p.stepSimulation()
        if not args.arnes:
            return
        pelvis = p.getBasePositionAndOrientation(atlas)[0]
        arnes["pies0"] = np.array(pelvis) + pies_relativos({n: 0.0 for n in J})
        if arnes["id"] is None:
            arnes["id"] = p.createConstraint(atlas, -1, -1, -1, p.JOINT_FIXED,
                                             [0, 0, 0], [0, 0, 0], pelvis)
        p.changeConstraint(arnes["id"], pelvis, [0, 0, 0, 1], maxForce=6000)

    def resetear_robot():
        p.resetBasePositionAndOrientation(atlas, POS_ATLAS, [0, 0, 0, 1])
        p.resetBaseVelocity(atlas, [0, 0, 0], [0, 0, 0])
        for n, i in J.items():
            p.resetJointState(atlas, i, 0.0)
            objetivo[n] = comando[n] = 0.0
        asentar_y_colgar()

    asentar_y_colgar()

    # ---- entrada -----------------------------------------------------------
    consola = None
    sliders = None
    if args.port:
        consola = ConsolaESP32(args.port, args.baud)
        consola.start()
    else:
        print("[INFO] Sin --port: usa los sliders 'Pot' y el teclado en la ventana")
        if args.gui:
            sliders = [p.addUserDebugParameter(f"Pot{i+1}", 0, 4095, 2048) for i in range(3)]
    print(f"[INFO] Arnés virtual: {'ON' if args.arnes else 'OFF (física libre)'}")
    print("[INFO] Teclas: 1=Modo  2=Saludar  3=Cámara  4=Home  5=Reset")
    TECLAS = {ord("1"): "PINZA", ord("2"): "BRAZO", ord("3"): "MUNECA",
              ord("4"): "HOME", ord("5"): "RESET"}

    modo = 0
    camara = False
    saludo_t0 = None                      # tiempo de inicio del gesto de saludo
    pots_filt, pots_toma = None, None
    tomado = [False, False, False]
    caido = False

    def congelar():
        nonlocal pots_toma
        if pots_filt is not None:
            pots_toma = pots_filt.copy()
            tomado[:] = [False, False, False]

    textos = [-1, -1, -1, -1]

    def hud():
        if not args.gui:
            return
        nombre, movs = MODOS[modo]
        lineas = [f"MODO {modo + 1}/{len(MODOS)}: {nombre}"] + \
                 [f"Pot{k + 1}: {m[0]}" for k, m in enumerate(movs)]
        for k, txt in enumerate(lineas):
            textos[k] = p.addUserDebugText(sin_tildes(txt), [-0.85, 2.05, 0.6 - 0.08 * k],
                                           [0.9, 0.1, 0.1] if k == 0 else [0, 0, 0],
                                           1.3 if k == 0 else 1.1, replaceItemUniqueId=textos[k])

    def imprimir_modo():
        nombre, movs = MODOS[modo]
        print(f"[MODO] {modo + 1}: {nombre} -> " + ", ".join(f"Pot{k+1}={m[0]}" for k, m in enumerate(movs)))

    def mostrar_vistas(on):
        """Paneles RGB / profundidad / segmentación de la cámara de la cabeza."""
        if args.gui:
            for v in (p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                      p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
                p.configureDebugVisualizer(v, 1 if on else 0)

    mostrar_vistas(False)
    hud()
    imprimir_modo()

    t0 = time.time()
    paso = 0
    cada = HZ_SIM // HZ_CTRL
    dt = 1.0 / HZ_CTRL
    duracion = int(args.duration_sec * HZ_SIM) if args.duration_sec > 0 else None
    try:
        while duracion is None or paso < duracion:
            # ---------------- 1) botones ---------------------------------
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
                if ev == "PINZA":                       # GPIO25: siguiente modo
                    modo = (modo + 1) % len(MODOS)
                    congelar()
                    imprimir_modo()
                    hud()
                elif ev == "BRAZO":                     # GPIO26: saludar
                    saludo_t0 = paso / HZ_SIM
                    print("[BTN] Saludo")
                elif ev == "MUNECA":                    # GPIO27: cámara sintética
                    camara = not camara
                    mostrar_vistas(camara)
                    print(f"[BTN] Cámara de la cabeza: {'ON' if camara else 'OFF'}")
                elif ev == "HOME":                      # GPIO33: postura inicial
                    for n in objetivo:
                        objetivo[n] = 0.0
                    congelar()
                    print("[BTN] Postura inicial")
                elif ev == "RESET":                     # GPIO32: levantar el robot
                    resetear_robot()
                    congelar()
                    caido = False
                    print("[BTN] Robot reiniciado")

            if paso % cada == 0:
                # ------------- 2) perillas -> objetivos del modo activo ----
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
                        tomado[:] = [True, True, True]
                    for k, (_, arts) in enumerate(MODOS[modo][1]):
                        if not tomado[k] and abs(pots_filt[k] - pots_toma[k]) > UMBRAL_TOMA:
                            tomado[k] = True
                        if tomado[k]:
                            for art, (lo, hi) in arts.items():
                                objetivo[art] = mapear(pots_filt[k], lo, hi)

                # ------------- 3) gesto de saludo (brazo derecho) ----------
                destino = dict(objetivo)
                if saludo_t0 is not None:
                    t = paso / HZ_SIM - saludo_t0
                    if t < 4.0:
                        destino["r_arm_shx"] = -0.3                          # brazo arriba
                        destino["r_arm_elx"] = -1.6                          # codo doblado
                        destino["r_arm_shz"] = 0.35 * math.sin(2 * math.pi * 1.2 * t)  # vaivén
                    else:
                        saludo_t0 = None

                # ------------- 4) limitar velocidad y respetar límites -----
                for n in comando:
                    lo, hi = LIM[n]
                    meta = float(np.clip(destino[n], lo, hi)) if hi > lo else 0.0
                    vmax = (VEL_MAX_PIERNAS if "_leg_" in n else VEL_MAX) * dt
                    comando[n] += float(np.clip(meta - comando[n], -vmax, vmax))
                    p.setJointMotorControl2(atlas, J[n], p.POSITION_CONTROL, comando[n],
                                            force=FUERZA[n])

                # ------------- 5) el arnés sigue a la pelvis ----------------
                if arnes["id"] is not None:
                    pelvis_meta = arnes["pies0"] - pies_relativos(comando)
                    p.changeConstraint(arnes["id"], pelvis_meta.tolist(), [0, 0, 0, 1], maxForce=6000)

                # ------------- 6) cámara sintética desde la cabeza ---------
                if camara and args.gui and paso % (HZ_SIM // 8) == 0:
                    est = p.getLinkState(atlas, J["neck_ry"])
                    R = np.array(p.getMatrixFromQuaternion(est[5])).reshape(3, 3)
                    ojo = np.array(est[4]) + R @ [0.15, 0, 0.05]
                    vista = p.computeViewMatrix(ojo.tolist(), (ojo + R @ [1, 0, 0]).tolist(),
                                                (R @ [0, 0, 1]).tolist())
                    proy = p.computeProjectionMatrixFOV(70, 1.5, 0.05, 30)
                    p.getCameraImage(320, 213, vista, proy, renderer=p.ER_BULLET_HARDWARE_OPENGL)

                # ------------- 7) ¿se cayó? (solo sin arnés) ---------------
                pos, orn = p.getBasePositionAndOrientation(atlas)
                rp = p.getEulerFromQuaternion(orn)
                if not caido and (abs(rp[0]) > 0.8 or abs(rp[1]) > 0.8 or pos[2] < -1.3):
                    caido = True
                    print("[AVISO] ¡El Atlas se cayó! Pulsa RESET (GPIO32) para levantarlo.")

            p.stepSimulation()
            paso += 1
            if args.gui or consola:
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
        for cli in (fantasma_cli, 0):
            try:
                p.disconnect(physicsClientId=cli)
            except p.error:
                pass


if __name__ == "__main__":
    booleano = lambda s: str(s).lower() in ("1", "true", "si", "yes")
    ap = argparse.ArgumentParser(description="Consola ESP32 para el robot Atlas")
    ap.add_argument("--port", default=None, help="Puerto de la ESP32 (ej. COM5). Vacío = sliders")
    ap.add_argument("--baud", default=115200, type=int)
    ap.add_argument("--gui", default=True, type=booleano)
    ap.add_argument("--arnes", default=True, type=booleano,
                    help="1 = arnés virtual (no se cae). 0 = física libre")
    ap.add_argument("--duration_sec", default=0, type=float)
    main(ap.parse_args())
