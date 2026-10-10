// Diagnostic: find the ESP32 pins wired to the EC200U main UART.
// Phase 1 reads every free GPIO with pull-down and pull-up: a pin that stays
// HIGH either way is being driven (or pulled up by a level shifter), as the
// modem's UART lines are. Modem TX turned out to be GPIO12.
// Phase 2 keeps RX on GPIO12 and tries each TX pin in turn, sending "AT" at
// 115200 (the modem's AT+IPR). After each try it listens for "RDY" so a pin
// that resets the modem is identified instead of corrupting later results.
// The modem restarts whenever the ESP32 resets, hence the initial wait.

#define MODEM_TX_ON_ESP_RX 12
#define BOOT_WAIT_MS 20000
#define REBOOT_LISTEN_MS 12000

const int candidates[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21,
  33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 47, 48
};  // skips 0/45/46 (strapping), 19/20 (USB), 26-32 (flash/PSRAM)
const int numCandidates = sizeof(candidates) / sizeof(candidates[0]);

// Pulled-high pins from phase 1 first, then the rest pins not yet ruled out.
const int txOrder[] = {2, 13, 10, 11, 3, 4, 5, 6, 7, 8, 9, 14, 15, 16, 17, 18, 21, 33, 34, 35, 36, 37};
const int numTx = sizeof(txOrder) / sizeof(txOrder[0]);

void printPrintable(const String &s) {
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    Serial.print(c >= 32 && c <= 126 ? c : '.');
  }
}

String readFor(unsigned long ms) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < ms) {
    while (Serial1.available()) resp += (char)Serial1.read();
  }
  return resp;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Phase 1: pins held HIGH with both pull-down and pull-up");
  for (int i = 0; i < numCandidates; i++) {
    int pin = candidates[i];
    pinMode(pin, INPUT_PULLDOWN);
    delay(5);
    int pd = digitalRead(pin);
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    int pu = digitalRead(pin);
    pinMode(pin, INPUT);
    if (pd && pu) Serial.printf("  GPIO%d\n", pin);
  }

  Serial.printf("Waiting %d s for the modem to boot...\n", BOOT_WAIT_MS / 1000);
  Serial1.begin(115200, SERIAL_8N1, MODEM_TX_ON_ESP_RX, -1);
  String boot = readFor(BOOT_WAIT_MS);
  Serial.print("  boot messages: ");
  printPrintable(boot);
  Serial.println();
  Serial1.end();

  Serial.println("Phase 2: TX probe, RX=GPIO12 at 115200");
  for (int t = 0; t < numTx; t++) {
    int tx = txOrder[t];
    Serial1.begin(115200, SERIAL_8N1, MODEM_TX_ON_ESP_RX, tx);
    delay(20);
    while (Serial1.available()) Serial1.read();
    Serial1.print("AT\r\n");
    String resp = readFor(500);
    Serial1.end();
    pinMode(tx, INPUT);

    Serial.printf("TX=GPIO%-2d: [", tx);
    printPrintable(resp);
    Serial.print("]");
    if (resp.indexOf("OK") >= 0) {
      Serial.println("  <== MODEM RX FOUND");
      break;
    }

    // Listen with TX released to see whether that pin restarted the modem.
    Serial1.begin(115200, SERIAL_8N1, MODEM_TX_ON_ESP_RX, -1);
    String after = readFor(REBOOT_LISTEN_MS);
    Serial1.end();
    Serial.println(after.indexOf("RDY") >= 0 ? "  !! modem rebooted after this pin" : "");
    if (after.indexOf("RDY") >= 0) delay(5000);  // let it finish booting
  }
  Serial.println("--- probe complete ---");
}

void loop() {}
