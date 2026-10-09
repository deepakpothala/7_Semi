// Passive RX sweep listening for periodic "+QIND":"csq" URCs, which the
// modem should emit every few seconds on any active AT-capable port,
// including the Main UART if it's truly wired to one of these pins.
const int candidates[] = {
  1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21,
  33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 47, 48, 0, 3, 45, 46
};
const int numCandidates = sizeof(candidates) / sizeof(candidates[0]);
const long bauds[] = {9600, 115200, 460800, 921600};
const int numBauds = sizeof(bauds) / sizeof(bauds[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("CSQ URC sweep starting (longer dwell per pin)...");
}

void loop() {
  for (int i = 0; i < numCandidates; i++) {
    int pin = candidates[i];
    for (int b = 0; b < numBauds; b++) {
      Serial1.begin(bauds[b], SERIAL_8N1, pin, -1);
      delay(30);
      while (Serial1.available()) Serial1.read();

      unsigned long start = millis();
      String resp = "";
      while (millis() - start < 1200) {
        while (Serial1.available()) {
          resp += (char)Serial1.read();
        }
      }

      if (resp.length() > 0) {
        Serial.printf("GPIO%-3d Baud=%-7ld: %d bytes: [", pin, bauds[b], resp.length());
        for (size_t j = 0; j < resp.length() && j < 150; j++) {
          char c = resp[j];
          if (c >= 32 && c <= 126) Serial.print(c);
          else Serial.print('.');
        }
        Serial.println("]");
      }
      Serial1.end();
      delay(20);
    }
  }
  Serial.println("--- sweep complete ---");
}
