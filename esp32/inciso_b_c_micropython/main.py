# ================================================================
# Práctica Real-to-Sim — Inciso b)  |  ESP32 + MicroPython (Thonny)
# Consola de mandos para el robot Baxter: 3 potenciómetros + 5 pulsadores.
# Guárdalo en la ESP32 como  main.py  para que arranque solo.
#
# POTENCIÓMETROS (patas de los extremos a 3V3 y GND, pata central al GPIO)
#   Pot 1   GPIO34        X  -> adelante / atrás
#   Pot 2   GPIO35        Y  -> izquierda / derecha
#   Pot 3   GPIO36 (VP)   Z  -> subir / bajar   (o giro de muñeca en modo MUÑECA)
#
# PULSADORES (una pata al GPIO, la otra a GND; pull-up interno)
#   GPIO25  PINZA    abrir / cerrar la pinza
#   GPIO26  BRAZO    cambiar brazo izquierdo <-> derecho
#   GPIO27  MUNECA   el pot 3 pasa de altura a giro de muñeca (y viceversa)
#   GPIO33  HOME     el brazo activo vuelve a su posición de reposo
#   GPIO32  RESET    los cubos vuelven a su lugar
#
# Mensajes que envía por USB al PC (115200 baudios):
#   POT:x,y,z   (0..4095, 30 veces por segundo)
#   BTN:NOMBRE  (cuando se pulsa un botón)
# ================================================================
from machine import Pin, ADC
import time

# ---------- potenciómetros ----------
pots = []
for gpio in (34, 35, 36):
    adc = ADC(Pin(gpio))
    adc.atten(ADC.ATTN_11DB)          # rango completo 0 – 3.3 V
    pots.append(adc)


def leer(adc):
    """Promedio de 8 lecturas, escalado a 0..4095."""
    s = 0
    for _ in range(8):
        try:
            s += adc.read_u16() >> 4   # MicroPython reciente (0..65535 -> 0..4095)
        except AttributeError:
            s += adc.read()            # versiones antiguas (0..4095)
    return s // 8


ALFA = 0.3                             # filtro pasa-bajos (0..1): menor = más suave
filtrado = [float(leer(a)) for a in pots]

# ---------- pulsadores ----------
BOTONES = [
    ("PINZA", Pin(25, Pin.IN, Pin.PULL_UP)),
    ("BRAZO", Pin(26, Pin.IN, Pin.PULL_UP)),
    ("MUNECA", Pin(27, Pin.IN, Pin.PULL_UP)),
    ("HOME", Pin(33, Pin.IN, Pin.PULL_UP)),
    ("RESET", Pin(32, Pin.IN, Pin.PULL_UP)),
]
anterior = [1] * len(BOTONES)
ANTIRREBOTE_MS = 40

led = Pin(2, Pin.OUT)                  # LED azul integrado
PERIODO_POT_MS = 33                    # ~30 Hz
ultimo_envio = time.ticks_ms()

print("ESP32 lista: consola Baxter")

while True:
    # --- botones: flanco de bajada con antirrebote ---
    for i, (nombre, pin) in enumerate(BOTONES):
        estado = pin.value()
        if anterior[i] == 1 and estado == 0:
            time.sleep_ms(ANTIRREBOTE_MS)
            if pin.value() == 0:
                print("BTN:" + nombre)
                led.value(not led.value())
        anterior[i] = estado

    # --- potenciómetros: filtrar y enviar cada 33 ms ---
    ahora = time.ticks_ms()
    if time.ticks_diff(ahora, ultimo_envio) >= PERIODO_POT_MS:
        ultimo_envio = ahora
        for k in range(3):
            filtrado[k] += ALFA * (leer(pots[k]) - filtrado[k])
        print("POT:%d,%d,%d" % (filtrado[0], filtrado[1], filtrado[2]))

    time.sleep_ms(2)
