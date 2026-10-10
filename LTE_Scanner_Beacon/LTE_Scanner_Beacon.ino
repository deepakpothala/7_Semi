// 7Semi ESP32-S3 EC200U: LTE network scanner -> BLE beacon.
// The EC200U reports the serving LTE cell and its neighbour cells
// (AT+QENG) plus the operators it can see (AT+COPS=?). The ESP32-S3 packs
// each result into a BLE advertisement and rotates through them, together
// with a status advert describing the modem's state.
// LTE_Scanner_Display (LilyGO T-Display-S3) listens for these adverts.
//
// Reliability: waits for the modem (AT, SIM, registration) before scanning,
// re-initialises it if it reboots ("RDY") or stops answering, and a task
// watchdog restarts the ESP32 if the modem task or advert loop hangs.
//
// Built for Arduino-ESP32 core 3.x (its built-in BLE library).
// Advert format: see README.md "BLE advert format".

#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <esp_task_wdt.h>
#include <functional>

#define MODEM_RX_PIN 12  // ESP32 RX <- EC200U TX (manual says 16; measured 12)
#define MODEM_TX_PIN 13  // ESP32 TX -> EC200U RX (manual says 17; measured 13)
#define MODEM_BAUD 115200

#define CELL_SCAN_INTERVAL_MS 5000
#define OPERATOR_SCAN_INTERVAL_MS (10UL * 60 * 1000)
#define OPERATOR_RETRY_MS (60UL * 1000)
#define ADV_DWELL_MS 500  // how long each record is advertised
#define WDT_TIMEOUT_MS 30000
#define MAX_AT_TIMEOUTS 3  // consecutive timeouts before re-initialising the modem

#define COMPANY_ID 0xFFFF  // Bluetooth SIG ID reserved for testing
#define MAGIC0 '7'
#define MAGIC1 'S'
#define TYPE_CELL 1
#define TYPE_OPERATOR 2
#define TYPE_STATUS 3

#define MODEM_STARTING 0
#define MODEM_NO_SIM 1
#define MODEM_SEARCHING 2
#define MODEM_REGISTERED 3
#define MODEM_NOT_RESPONDING 4

#define STATUS_FLAG_OPS_SCAN 0x01  // AT+COPS=? in progress

#define KIND_SERVING 0
#define KIND_INTRA 1  // neighbour on the same frequency
#define KIND_INTER 2  // neighbour on another frequency

#define MAX_CELLS 12
#define MAX_OPERATORS 8
#define OP_NAME_LEN 11  // longest name that fits in a 31-byte advert

struct Cell {
  uint16_t mcc, mnc;
  bool mnc3, tdd;
  uint8_t kind, band;
  uint16_t earfcn, pci;
  int8_t rsrp, rsrq, sinr;
  uint32_t cellId;
};

struct Operator {
  uint16_t mcc, mnc;
  bool mnc3;
  uint8_t stat, act;  // 3GPP 27.007: stat 1=available 2=current 3=forbidden, act 7=LTE
  char name[OP_NAME_LEN + 1];
};

static Cell cells[MAX_CELLS];
static int numCells = 0;
static uint8_t cellSeq = 0;
static Operator ops[MAX_OPERATORS];
static int numOps = 0;
static uint8_t opSeq = 0;
static SemaphoreHandle_t dataLock;
static BLEAdvertising *adv;

// Modem status, written by the modem task and read when building adverts.
static volatile uint8_t modemState = MODEM_STARTING;
static volatile uint8_t csq = 99;  // 99 = unknown
static volatile bool opsScanRunning = false;
static volatile unsigned long lastCellOk = 0, lastOpsOk = 0;  // millis(), 0 = never
static volatile uint8_t modemRecoveries = 0;
static uint8_t statusSeq = 0;

// Modem health, only touched by the modem task.
static int atTimeouts = 0;
static bool modemRebooted = false;

// ---------- Modem ----------

// Discards input until the line has been quiet for idleMs, so a late reply to
// an earlier command can't be mistaken for the next command's result.
// Still watches for "RDY", which the modem prints after it reboots.
void drainModem(unsigned long idleMs) {
  uint32_t window = 0;
  unsigned long lastByte = millis();
  while (millis() - lastByte < idleMs) {
    if (Serial1.available()) {
      window = (window << 8) | (uint8_t)Serial1.read();
      if ((window & 0xFFFFFF) == (('R' << 16) | ('D' << 8) | 'Y')) modemRebooted = true;
      lastByte = millis();
    } else {
      esp_task_wdt_reset();
      vTaskDelay(1);
    }
  }
}

// Sends an AT command and collects the reply until "OK", "ERROR" or timeout.
String sendAT(const String &cmd, unsigned long timeoutMs = 2000) {
  drainModem(20);
  Serial1.print(cmd);
  Serial1.print("\r\n");

  String resp = "";
  unsigned long start = millis();
  bool done = false;
  while (!done && millis() - start < timeoutMs) {
    while (Serial1.available()) {
      resp += (char)Serial1.read();
    }
    done = resp.indexOf("OK\r\n") >= 0 || resp.indexOf("ERROR") >= 0;
    if (!done) {
      esp_task_wdt_reset();  // a long AT+COPS=? is slow, not hung
      vTaskDelay(1);
    }
  }
  if (done) {
    atTimeouts = 0;
  } else {
    atTimeouts++;
    Serial.println("AT timeout: " + cmd);
    drainModem(1000);  // let the late reply arrive and throw it away
  }
  if (resp.indexOf("RDY") >= 0) modemRebooted = true;
  resp.trim();
  return resp;
}

// Splits on commas outside quotes, dropping the quotes. Returns field count.
int splitCsv(const String &s, String out[], int maxFields) {
  int n = 0;
  String cur = "";
  bool inQuotes = false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') {
      inQuotes = !inQuotes;
    } else if (c == ',' && !inQuotes) {
      if (n < maxFields) out[n++] = cur;
      cur = "";
    } else {
      cur += c;
    }
  }
  if (n < maxFields) out[n++] = cur;
  for (int i = 0; i < n; i++) out[i].trim();
  return n;
}

int findField(String f[], int n, const char *value) {
  for (int i = 0; i < n; i++) {
    if (f[i] == value) return i;
  }
  return -1;
}

int8_t clamp8(long v) {
  return (int8_t)constrain(v, -128, 127);
}

// LTE bands supported by the EC200U-CN.
uint8_t earfcnToBand(uint32_t e) {
  static const struct { uint8_t band; uint32_t lo, hi; } table[] = {
    {1, 0, 599}, {3, 1200, 1949}, {5, 2400, 2649}, {8, 3450, 3799},
    {34, 36200, 36349}, {38, 37750, 38249}, {39, 38250, 38649},
    {40, 38650, 39649}, {41, 39650, 41589},
  };
  for (auto &b : table) {
    if (e >= b.lo && e <= b.hi) return b.band;
  }
  return 0;
}

// Calls fn for each line of resp that starts with prefix.
void forEachLine(const String &resp, const char *prefix,
                 std::function<void(const String &)> fn) {
  int from = 0;
  while (from < (int)resp.length()) {
    int eol = resp.indexOf('\n', from);
    if (eol < 0) eol = resp.length();
    String line = resp.substring(from, eol);
    line.trim();
    if (line.startsWith(prefix)) fn(line.substring(strlen(prefix)));
    from = eol + 1;
  }
}

void scanCells() {
  Cell found[MAX_CELLS];
  int n = 0;

  // +QENG: "servingcell",<state>,"LTE",<is_tdd>,<MCC>,<MNC>,<cellID>,<PCID>,
  //        <earfcn>,<band>,<UL_bw>,<DL_bw>,<TAC>,<RSRP>,<RSRQ>,<RSSI>,<SINR>,...
  String resp = sendAT("AT+QENG=\"servingcell\"", 10000);
  forEachLine(resp, "+QENG:", [&](const String &line) {
    String f[20];
    int nf = splitCsv(line, f, 20);
    int i = findField(f, nf, "LTE");
    if (i < 0 || nf <= i + 14 || n >= MAX_CELLS) return;
    Cell &c = found[n++];
    c.kind = KIND_SERVING;
    c.tdd = f[i + 1] == "TDD";
    c.mcc = f[i + 2].toInt();
    c.mnc = f[i + 3].toInt();
    c.mnc3 = f[i + 3].length() >= 3;
    c.cellId = strtoul(f[i + 4].c_str(), nullptr, 16);
    c.pci = f[i + 5].toInt();
    c.earfcn = f[i + 6].toInt();
    c.band = f[i + 7].toInt();
    if (c.band == 0) c.band = earfcnToBand(c.earfcn);
    c.rsrp = clamp8(f[i + 11].toInt());
    c.rsrq = clamp8(f[i + 12].toInt());
    c.sinr = clamp8(f[i + 14].toInt());
  });

  if (n == 0) {
    Serial.println("No LTE serving cell (not registered?): " + resp);
  } else {
    // Neighbours are configured by the serving operator, so they share its MCC/MNC.
    // +QENG: "neighbourcell intra","LTE",<earfcn>,<PCID>,<RSRQ>,<RSRP>,<RSSI>,<SINR>,...
    const Cell serving = found[0];
    resp = sendAT("AT+QENG=\"neighbourcell\"", 10000);
    forEachLine(resp, "+QENG:", [&](const String &line) {
      String f[20];
      int nf = splitCsv(line, f, 20);
      int i = findField(f, nf, "LTE");
      if (i < 0 || nf <= i + 4 || n >= MAX_CELLS) return;
      Cell c = serving;
      c.kind = f[0].indexOf("inter") >= 0 ? KIND_INTER : KIND_INTRA;
      c.earfcn = f[i + 1].toInt();
      c.pci = f[i + 2].toInt();
      long a = f[i + 3].toInt(), b = f[i + 4].toInt();
      // Quectel documents RSRQ then RSRP; RSRP is always below -40 dBm, so
      // swap if the firmware reports them the other way round.
      if (a < -40) { long t = a; a = b; b = t; }
      c.rsrq = clamp8(a);
      c.rsrp = clamp8(b);
      c.sinr = nf > i + 6 ? clamp8(f[i + 6].toInt()) : 0;
      c.band = earfcnToBand(c.earfcn);
      c.tdd = c.band >= 33;
      c.cellId = 0;
      if (c.earfcn == serving.earfcn && c.pci == serving.pci) return;
      // Drop garbled lines: valid RSRP is -140..-44 dBm, RSRQ -34..3 dB.
      if (c.earfcn == 0 || b < -140 || b > -44 || a < -34 || a > 3) return;
      found[n++] = c;
    });
  }

  xSemaphoreTake(dataLock, portMAX_DELAY);
  memcpy(cells, found, sizeof(Cell) * n);
  numCells = n;
  cellSeq++;
  xSemaphoreGive(dataLock);
  if (n > 0) lastCellOk = millis();

  for (int k = 0; k < n; k++) {
    const Cell &c = found[k];
    Serial.printf("%s %u-%0*u B%u EARFCN %u PCI %u RSRP %d RSRQ %d SINR %d\n",
                  c.kind == KIND_SERVING ? "SERV " : c.kind == KIND_INTRA ? "INTRA" : "INTER",
                  c.mcc, c.mnc3 ? 3 : 2, c.mnc, c.band, c.earfcn, c.pci, c.rsrp, c.rsrq, c.sinr);
  }
}

bool scanOperators() {
  Serial.println("Scanning operators (AT+COPS=?), this takes up to 3 minutes...");
  // +COPS: (2,"Airtel","Airtel","40445",7),(1,"Jio 4G","Jio 4G","405857",7),,(0-4),(0-2)
  String resp = sendAT("AT+COPS=?", 180000);
  int p = resp.indexOf("+COPS:");
  if (p < 0) {
    Serial.println("COPS scan failed: " + resp);
    return false;
  }

  Operator found[MAX_OPERATORS];
  int n = 0;
  int open;
  while (n < MAX_OPERATORS && (open = resp.indexOf('(', p)) >= 0) {
    int close = resp.indexOf(')', open);
    if (close < 0) break;
    String group = resp.substring(open + 1, close);
    p = close + 1;
    if (group.indexOf('"') < 0) continue;  // trailing (0-4),(0-2) ranges

    String f[5];
    if (splitCsv(group, f, 5) < 5 || f[3].length() < 5) continue;
    Operator &o = found[n++];
    o.stat = f[0].toInt();
    o.mcc = f[3].substring(0, 3).toInt();
    o.mnc = f[3].substring(3).toInt();
    o.mnc3 = f[3].length() >= 6;
    o.act = f[4].toInt();
    const String &name = f[2].length() ? f[2] : f[1];
    strncpy(o.name, name.c_str(), OP_NAME_LEN);
    o.name[OP_NAME_LEN] = '\0';
  }

  xSemaphoreTake(dataLock, portMAX_DELAY);
  memcpy(ops, found, sizeof(Operator) * n);
  numOps = n;
  opSeq++;
  xSemaphoreGive(dataLock);
  if (n > 0) lastOpsOk = millis();

  for (int k = 0; k < n; k++) {
    Serial.printf("OP %-11s %u-%0*u stat %u act %u\n", found[k].name, found[k].mcc,
                  found[k].mnc3 ? 3 : 2, found[k].mnc, found[k].stat, found[k].act);
  }
  return n > 0;
}

void clearCells() {
  xSemaphoreTake(dataLock, portMAX_DELAY);
  if (numCells) cellSeq++;
  numCells = 0;
  xSemaphoreGive(dataLock);
}

// Waits for the modem to answer AT (it takes ~10 s after power-up), then
// applies the settings a modem reboot would have reset.
bool modemInit() {
  modemState = MODEM_STARTING;
  Serial.println("Waiting for modem...");
  bool alive = false;
  for (int i = 0; i < 20 && !alive; i++) {
    alive = sendAT("AT", 1000).indexOf("OK") >= 0;
  }
  if (!alive) {
    modemState = MODEM_NOT_RESPONDING;
    return false;
  }
  sendAT("ATE0");
  sendAT("AT+CMEE=1");  // numeric error codes
  modemRebooted = false;
  atTimeouts = 0;
  Serial.println("Modem ready");
  return true;
}

// Checks SIM, signal and registration. Returns true when registered on LTE.
bool updateStatus() {
  String r = sendAT("AT+CPIN?");
  if (r.indexOf("READY") < 0) {
    modemState = atTimeouts ? MODEM_NOT_RESPONDING : MODEM_NO_SIM;
    return false;
  }

  r = sendAT("AT+CSQ");  // +CSQ: <rssi>,<ber>
  int p = r.indexOf("+CSQ:");
  if (p >= 0) csq = r.substring(p + 5).toInt();

  r = sendAT("AT+CEREG?");  // +CEREG: <n>,<stat>; stat 1 = home, 5 = roaming
  int stat = -1;
  p = r.indexOf("+CEREG:");
  if (p >= 0) {
    int comma = r.indexOf(',', p);
    if (comma > 0) stat = r.substring(comma + 1).toInt();
  }
  bool registered = stat == 1 || stat == 5;
  modemState = registered ? MODEM_REGISTERED : MODEM_SEARCHING;
  return registered;
}

void modemTask(void *) {
  esp_task_wdt_add(NULL);
  bool ready = false;
  bool haveOps = false;
  unsigned long lastOpsAttempt = 0;

  for (;;) {
    esp_task_wdt_reset();

    if (!ready) {
      clearCells();
      ready = modemInit();
      if (!ready) {
        Serial.println("Modem not responding, retrying in 5 s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    if (updateStatus()) {
      scanCells();
      unsigned long wait = haveOps ? OPERATOR_SCAN_INTERVAL_MS : OPERATOR_RETRY_MS;
      if (lastOpsAttempt == 0 || millis() - lastOpsAttempt > wait) {
        opsScanRunning = true;
        haveOps = scanOperators();
        opsScanRunning = false;
        lastOpsAttempt = millis();
      }
    } else {
      Serial.printf("Modem state %u, not scanning\n", modemState);
      clearCells();
    }

    if (modemRebooted || atTimeouts >= MAX_AT_TIMEOUTS) {
      Serial.println(modemRebooted ? "Modem rebooted, re-initialising"
                                   : "Modem stopped answering, re-initialising");
      ready = false;
      modemRecoveries++;
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(CELL_SCAN_INTERVAL_MS));
  }
}

// ---------- BLE ----------

int writeHeader(uint8_t *p, uint8_t type, uint8_t seq, uint8_t idx, uint8_t total) {
  p[0] = COMPANY_ID & 0xFF;
  p[1] = COMPANY_ID >> 8;
  p[2] = MAGIC0;
  p[3] = MAGIC1;
  p[4] = type;
  p[5] = seq;
  p[6] = idx;
  p[7] = total;
  return 8;
}

void put16(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = v >> 8;
}

int buildCellPacket(uint8_t *p, int idx) {
  const Cell &c = cells[idx];
  int i = writeHeader(p, TYPE_CELL, cellSeq, idx, numCells);
  put16(p + i, c.mcc); i += 2;
  put16(p + i, c.mnc); i += 2;
  p[i++] = (c.kind & 0x03) | (c.mnc3 ? 0x04 : 0) | (c.tdd ? 0x08 : 0);
  p[i++] = c.band;
  put16(p + i, c.earfcn); i += 2;
  put16(p + i, c.pci); i += 2;
  p[i++] = (uint8_t)c.rsrp;
  p[i++] = (uint8_t)c.rsrq;
  p[i++] = (uint8_t)c.sinr;
  for (int b = 0; b < 4; b++) p[i++] = (c.cellId >> (8 * b)) & 0xFF;
  return i;  // 25 bytes
}

int buildOperatorPacket(uint8_t *p, int idx) {
  const Operator &o = ops[idx];
  int i = writeHeader(p, TYPE_OPERATOR, opSeq, idx, numOps);
  put16(p + i, o.mcc); i += 2;
  put16(p + i, o.mnc); i += 2;
  p[i++] = o.mnc3 ? 0x04 : 0;
  p[i++] = o.stat;
  p[i++] = o.act;
  int len = strlen(o.name);
  memcpy(p + i, o.name, len);
  return i + len;  // up to 24 bytes
}

uint16_t ageSeconds(unsigned long t, unsigned long now) {
  if (t == 0) return 0xFFFF;  // never
  return (uint16_t)min((now - t) / 1000, 0xFFFEUL);
}

int buildStatusPacket(uint8_t *p) {
  unsigned long now = millis();
  int i = writeHeader(p, TYPE_STATUS, statusSeq++, 0, 1);
  p[i++] = modemState;
  p[i++] = opsScanRunning ? STATUS_FLAG_OPS_SCAN : 0;
  p[i++] = csq;
  put16(p + i, ageSeconds(lastCellOk, now)); i += 2;
  put16(p + i, ageSeconds(lastOpsOk, now)); i += 2;
  uint32_t uptime = now / 1000;
  for (int b = 0; b < 4; b++) p[i++] = (uptime >> (8 * b)) & 0xFF;
  p[i++] = modemRecoveries;
  return i;  // 20 bytes
}

void advertise(const uint8_t *data, int len) {
  BLEAdvertisementData ad;
  ad.setFlags(0x06);  // LE general discoverable, BR/EDR not supported
  ad.setManufacturerData(String((const char *)data, len));
  adv->stop();
  adv->setAdvertisementData(ad);
  adv->start();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(2000);
  Serial.println("7Semi ESP32-S3 EC200U - LTE Scanner Beacon");

  dataLock = xSemaphoreCreateMutex();

  BLEDevice::init("7Semi-LTE");
  adv = BLEDevice::getAdvertising();
#if defined(CONFIG_NIMBLE_ENABLED)  // core 3.3+ builds the S3 with NimBLE
  adv->setAdvertisementType(BLE_GAP_CONN_MODE_NON);
#else
  adv->setAdvertisementType(ADV_TYPE_NONCONN_IND);
#endif
  adv->setMinInterval(0xA0);  // 100 ms (units of 0.625 ms)
  adv->setMaxInterval(0xB0);
  adv->setScanResponse(false);

  // Restart the ESP32 if the modem task or this loop stops checking in.
  esp_task_wdt_config_t wdt = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);  // setup() and loop() run in the same task

  // All modem I/O happens in its own task, including start-up.
  xTaskCreatePinnedToCore(modemTask, "modem", 12288, nullptr, 1, nullptr, 1);
}

void loop() {
  static int slot = 0;
  uint8_t pkt[25];
  int len;

  // Rotation: status first, then cells, then operators. Status is always
  // sent, so the display knows the beacon is alive even with no data.
  xSemaphoreTake(dataLock, portMAX_DELAY);
  int total = 1 + numCells + numOps;
  slot %= total;
  if (slot == 0) len = buildStatusPacket(pkt);
  else if (slot <= numCells) len = buildCellPacket(pkt, slot - 1);
  else len = buildOperatorPacket(pkt, slot - 1 - numCells);
  xSemaphoreGive(dataLock);

  advertise(pkt, len);
  slot++;
  esp_task_wdt_reset();
  delay(ADV_DWELL_MS);
}
