// Diagnostic: scan common baud rates and both pin orientations to find
// how the EC200U modem is actually wired/configured on this board.
// Sends "AT\r\n" at each combination and reports any bytes received.

const long bauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
const int numBauds = sizeof(bauds) / sizeof(bauds[0]);

struct PinSet {
  int rx, tx;
  const char* label;
};
PinSet pinSets[] = {
  {16, 17, "RX=16,TX=17 (per manual)"},
  {17, 16, "RX=17,TX=16 (swapped)"},
};
const int numPinSets = sizeof(pinSets) / sizeof(pinSets[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Baud/pin scanner starting...");
}

void loop() {
  for (int p = 0; p < numPinSets; p++) {
    for (int b = 0; b < numBauds; b++) {
      Serial1.begin(bauds[b], SERIAL_8N1, pinSets[p].rx, pinSets[p].tx);
      delay(100);

      // Clear any stale bytes
      while (Serial1.available()) Serial1.read();

      Serial1.print("AT\r\n");

      unsigned long start = millis();
      String resp = "";
      while (millis() - start < 500) {
        while (Serial1.available()) {
          resp += (char)Serial1.read();
        }
      }

      Serial.printf("Pins[%s] Baud=%-7ld : ", pinSets[p].label, bauds[b]);
      if (resp.length() == 0) {
        Serial.println("(no response)");
      } else {
        Serial.print("GOT ");
        Serial.print(resp.length());
        Serial.print(" bytes: [");
        for (size_t i = 0; i < resp.length(); i++) {
          char c = resp[i];
          if (c >= 32 && c <= 126) Serial.print(c);
          else {
            Serial.print("\\x");
            if ((uint8_t)c < 16) Serial.print("0");
            Serial.print((uint8_t)c, HEX);
          }
        }
        Serial.println("]");
      }

      Serial1.end();
      delay(100);
    }
  }
  Serial.println("--- Full scan complete, repeating in 5s ---");
  delay(5000);
}
