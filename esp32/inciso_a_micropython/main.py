# ================================================================
# Práctica Real-to-Sim — Inciso a)  |  ESP32 + MicroPython (Thonny)
# Consola de 5 botones que manda comandos por USB (UART) al PC.
# Guárdalo en la ESP32 como  main.py  para que arranque solo.
#
#   Botón   GPIO   Comando   Acción en la simulación
#   A       25     CMD:A     ir al punto A
#   B       26     CMD:B     ir al punto B
#   C       27     CMD:C     ir al punto C
#   SEQ     33     CMD:S     secuencia automática A -> B -> C (opcional)
#   LAND    32     CMD:L     aterrizar (opcional)
#
# Cableado de cada botón:  GPIO ---[botón]--- GND
# (se usa la resistencia pull-up interna, no hace falta resistencia externa)
# ================================================================
from machine import Pin
import time

BOTONES = {
    "A": Pin(25, Pin.IN, Pin.PULL_UP),
    "B": Pin(26, Pin.IN, Pin.PULL_UP),
    "C": Pin(27, Pin.IN, Pin.PULL_UP),
    "S": Pin(33, Pin.IN, Pin.PULL_UP),
    "L": Pin(32, Pin.IN, Pin.PULL_UP),
}
led = Pin(2, Pin.OUT)          # LED azul integrado de la mayoría de placas ESP32

ANTIRREBOTE_MS = 50
anterior = {k: 1 for k in BOTONES}   # 1 = suelto (pull-up)

print("ESP32 lista: consola de drones")

while True:
    for cmd, pin in BOTONES.items():
        estado = pin.value()
        if anterior[cmd] == 1 and estado == 0:          # flanco de bajada = pulsado
            time.sleep_ms(ANTIRREBOTE_MS)
            if pin.value() == 0:                        # sigue pulsado -> es real
                print("CMD:" + cmd)                     # viaja por USB al PC
                led.value(1)
                time.sleep_ms(80)
                led.value(0)
        anterior[cmd] = estado
    time.sleep_ms(5)
