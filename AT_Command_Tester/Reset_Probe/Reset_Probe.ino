// Diagnostic: find which ESP32 GPIO restarts the EC200U, and which GPIOs
// report modem status.
// During pin probing the modem rebooted when a pin in GPIO3-11 was driven
// HIGH for ~270 ms. GPIO9 reads as pulled low (like a transistor base), so
// it is the main suspect for a reset line. Each candidate is pulsed HIGH,
// then the modem is watched for "RDY" (it rebooted) and an AT reply.
// GPIO1 and GPIO2 are pulled high (like modem status outputs), so their
// level changes are logged during each test. Findings so far: pulsing GPIO1
// HIGH or LOW for 300 ms does nothing.

#define MODEM_RX_PIN 12
#define MODEM_TX_PIN 13
#define LISTEN_MS 25000

const int resetCandidates[] = {9, 10, 11};
const int statusPins[] = {1, 2};

bool modemAnswers() {
  while (Serial1.available()) Serial1.read();
  Serial1.print("AT\r\n");
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < 1000) {
    while (Serial1.available()) resp += (char)Serial1.read();
    if (resp.indexOf("OK") >= 0) return true;
  }
  return false;
}

// Listens for LISTEN_MS, logging RDY and any status-pin level changes.
// Returns true if RDY arrived.
bool listenForReboot() {
  String all = "";
  bool rebooted = false;
  int last[2] = {digitalRead(statusPins[0]), digitalRead(statusPins[1])};
  unsigned long start = millis();
  while (millis() - start < LISTEN_MS) {
    while (Serial1.available()) all += (char)Serial1.read();
    if (!rebooted && all.indexOf("RDY") >= 0) {
      Serial.printf("  %5lu ms: RDY\n", millis() - start);
      rebooted = true;
    }
    for (int i = 0; i < 2; i++) {
      int level = digitalRead(statusPins[i]);
      if (level != last[i]) {
        Serial.printf("  %5lu ms: GPIO%d -> %d\n", millis() - start, statusPins[i], level);
        last[i] = level;
      }
    }
    delay(2);
  }
  return rebooted;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  for (int pin : statusPins) pinMode(pin, INPUT);
  delay(3000);

  Serial.printf("Modem before test: AT %s\n", modemAnswers() ? "OK" : "NO ANSWER");
  Serial.println("Idle 5 s, watching status pins:");
  unsigned long t = millis();
  int last[2] = {digitalRead(1), digitalRead(2)};
  int toggles[2] = {0, 0};
  while (millis() - t < 5000) {
    for (int i = 0; i < 2; i++) {
      int level = digitalRead(statusPins[i]);
      if (level != last[i]) { toggles[i]++; last[i] = level; }
    }
    delay(2);
  }
  Serial.printf("  GPIO1 toggles: %d (now %d), GPIO2 toggles: %d (now %d)\n",
                toggles[0], last[0], toggles[1], last[1]);

  for (int pin : resetCandidates) {
    Serial.printf("Pulse GPIO%d HIGH for 300 ms (idle level %d)\n", pin, digitalRead(pin));
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delay(300);
    pinMode(pin, INPUT);
    bool rebooted = listenForReboot();
    delay(3000);
    Serial.printf("  result: %s, AT %s\n", rebooted ? "modem REBOOTED" : "no reboot",
                  modemAnswers() ? "OK" : "NO ANSWER");
    if (rebooted) break;  // found it; don't keep restarting the modem
  }
  Serial.println("--- reset probe complete ---");
}

void loop() {}
