#include <Arduino.h>

void setup() {
  // Inicializa a Serial com baud rate de 115200
  Serial.begin(115200);
  
  // Aguarda a porta serial conectar (importante para USB CDC nativo no ESP32-C3)
  delay(2000);
  
  Serial.println("\n--- REMUS ESP32 OK ---");
  Serial.println("Fase 1: Bring-up basico concluido.");
}

void loop() {
  Serial.println("running");
  delay(1000);
}
