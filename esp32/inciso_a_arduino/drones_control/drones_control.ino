// ================================================================
// Práctica Real-to-Sim — Inciso a)  |  ESP32 + Arduino/PlatformIO
// (Alternativa en C++ al main.py de MicroPython; usa SOLO una de las dos)
//
//   Botón   GPIO   Comando   Acción en la simulación
//   A       25     CMD:A     ir al punto A
//   B       26     CMD:B     ir al punto B
//   C       27     CMD:C     ir al punto C
//   SEQ     33     CMD:S     secuencia automática A -> B -> C (opcional)
//   LAND    32     CMD:L     aterrizar (opcional)
//
// Cableado de cada botón:  GPIO ---[botón]--- GND  (pull-up interno)
// ================================================================
const int  N = 5;
const int  PINES[N]    = {25, 26, 27, 33, 32};
const char COMANDOS[N] = {'A', 'B', 'C', 'S', 'L'};
const int  LED = 2;
const unsigned long ANTIRREBOTE_MS = 50;

int anterior[N];

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  for (int i = 0; i < N; i++) {
    pinMode(PINES[i], INPUT_PULLUP);
    anterior[i] = HIGH;
  }
  Serial.println("ESP32 lista: consola de drones");
}

void loop() {
  for (int i = 0; i < N; i++) {
    int estado = digitalRead(PINES[i]);
    if (anterior[i] == HIGH && estado == LOW) {      // flanco de bajada
      delay(ANTIRREBOTE_MS);
      if (digitalRead(PINES[i]) == LOW) {
        Serial.print("CMD:");
        Serial.println(COMANDOS[i]);                 // viaja por USB al PC
        digitalWrite(LED, HIGH);
        delay(80);
        digitalWrite(LED, LOW);
      }
    }
    anterior[i] = estado;
  }
  delay(5);
}
