// Pin donde está conectado el módulo de relé
const int relayPin = 8;

void setup() {
  // Configuramos el pin del relé como salida
  pinMode(relayPin, OUTPUT);

  // Al inicio dejamos el relé apagado (bomba apagada)
  digitalWrite(relayPin, LOW);

  // Iniciamos la comunicación serial
  Serial.begin(9600);
  Serial.println("Sistema listo.");
  Serial.println("Presiona 'O' para ENCENDER la bomba.");
  Serial.println("Presiona 'P' para APAGAR la bomba.");
}

void loop() {
  // Verificamos si llegó algún dato por el puerto serial
  if (Serial.available() > 0) {
    char tecla = Serial.read();  // Leemos el carácter recibido

    if (tecla == 'o' || tecla == 'O') {
      // Activar relé (bomba de vacío encendida)
      digitalWrite(relayPin, HIGH);
      Serial.println("Bomba ENCENDIDA.");
    }

    if (tecla == 'p' || tecla == 'P') {
      // Desactivar relé (bomba de vacío apagada)
      digitalWrite(relayPin, LOW);
      Serial.println("Bomba APAGADA.");
    }
  }
}
