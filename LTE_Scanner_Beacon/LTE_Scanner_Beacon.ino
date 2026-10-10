// 7Semi ESP32-S3 EC200U: LTE network scanner -> BLE beacon.
// The EC200U reports the serving LTE cell and its neighbour cells
// (AT+QENG) plus the operators it can see (AT+COPS=?). The ESP32-S3 packs
// each result into a BLE advertisement and rotates through them.
// LTE_Scanner_Display (LilyGO T-Display-S3) listens for these adverts.
//
// Built for Arduino-ESP32 core 3.x (its built-in BLE library).
// Advert format: see README.md "BLE advert format".

#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <functional>

#define MODEM_RX_PIN 12  // ESP32 RX <- EC200U TX (manual says 16; measured 12)
#define MODEM_TX_PIN 13  // ESP32 TX -> EC200U RX (manual says 17; measured 13)
#define MODEM_BAUD 115200

#define CELL_SCAN_INTERVAL_MS 5000
#define OPERATOR_SCAN_INTERVAL_MS (10UL * 60 * 1000)
#define OPERATOR_RETRY_MS (60UL * 1000)
#define ADV_DWELL_MS 500  // how long each record is advertised

#define COMPANY_ID 0xFFFF  // Bluetooth SIG ID reserved for testing
#define MAGIC0 '7'
#define MAGIC1 'S'
#define TYPE_CELL 1
#define TYPE_OPERATOR 2

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

// ---------- Modem ----------

// Discards input until the line has been quiet for idleMs, so a late reply to
// an earlier command can't be mistaken for the next command's result.
void drainModem(unsigned long idleMs) {
  unsigned long lastByte = millis();
  while (millis() - lastByte < idleMs) {
    if (Serial1.available()) {
      Serial1.read();
      lastByte = millis();
    } else {
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
    if (!done) vTaskDelay(1);
  }
  if (!done) {
    Serial.println("AT timeout: " + cmd);
    drainModem(1000);  // let the late reply arrive and throw it away
  }
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

  for (int k = 0; k < n; k++) {
    Serial.printf("OP %-11s %u-%0*u stat %u act %u\n", found[k].name, found[k].mcc,
                  found[k].mnc3 ? 3 : 2, found[k].mnc, found[k].stat, found[k].act);
  }
  return n > 0;
}

void modemTask(void *) {
  unsigned long lastOpsAttempt = 0;
  bool haveOps = false;
  for (;;) {
    scanCells();
    unsigned long wait = haveOps ? OPERATOR_SCAN_INTERVAL_MS : OPERATOR_RETRY_MS;
    if (lastOpsAttempt == 0 || millis() - lastOpsAttempt > wait) {
      haveOps = scanOperators();
      lastOpsAttempt = millis();
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

  sendAT("ATE0");
  sendAT("AT+CMEE=1");  // numeric error codes

  xTaskCreatePinnedToCore(modemTask, "modem", 12288, nullptr, 1, nullptr, 1);
}

void loop() {
  static int slot = 0;
  uint8_t pkt[25];
  int len = 0;

  xSemaphoreTake(dataLock, portMAX_DELAY);
  int total = numCells + numOps;
  if (total > 0) {
    slot %= total;
    len = slot < numCells ? buildCellPacket(pkt, slot) : buildOperatorPacket(pkt, slot - numCells);
  }
  xSemaphoreGive(dataLock);

  if (len > 0) {
    advertise(pkt, len);
    slot++;
  }
  delay(ADV_DWELL_MS);
}
