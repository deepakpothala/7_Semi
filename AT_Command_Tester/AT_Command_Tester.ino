// 7Semi ESP32-S3 EC200U board bring-up: AT command passthrough tester.
// Type AT commands into the Serial Monitor; they are forwarded to the
// EC200U modem over UART, and its responses are printed back.

#define MODEM_RX_PIN 16  // ESP32 RX <- EC200U TX
#define MODEM_TX_PIN 17  // ESP32 TX -> EC200U RX
#define MODEM_BAUD 115200

void setup() {
  Serial.begin(115200);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  delay(2000);
  Serial.println("7Semi ESP32-S3 EC200U - AT Command Tester");
  Serial.println("Type an AT command and press enter (e.g. AT, ATI, AT+QGPS=1)");
}

void loop() {
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
