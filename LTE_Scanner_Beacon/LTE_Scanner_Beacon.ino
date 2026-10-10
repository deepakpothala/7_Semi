// 7Semi ESP32-S3 EC200U: LTE network scanner -> BLE beacon.
// The EC200U reports the serving LTE cell and its neighbour cells
// (AT+QENG) plus every operator in range with its cells' signal (AT+QOPS).
// Once a second the ESP32-S3 packs everything (modem status, cells,
// networks) into one snapshot and broadcasts it as a BLE 5 extended
// advertisement at +20 dBm. LTE_Scanner_Display (LilyGO T-Display-S3)
// listens for it.
//
// Reliability: waits for the modem (AT, SIM, registration) before scanning,
// re-initialises it if it reboots ("RDY") or stops answering, hard-resets it
// through GPIO9 if it won't answer at all, and a task watchdog restarts the
// ESP32 if the modem task or advert loop hangs.
//
// Built for Arduino-ESP32 core 3.x with the NimBLE-Arduino library (2.x),
// which must have CONFIG_BT_NIMBLE_EXT_ADV enabled in its nimconfig.h.
// Advert format: see README.md "BLE advert format".

#include <NimBLEDevice.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include <functional>

#if !CONFIG_BT_NIMBLE_EXT_ADV
#error "Enable CONFIG_BT_NIMBLE_EXT_ADV in NimBLE-Arduino's src/nimconfig.h"
#endif

#define MODEM_RX_PIN 12  // ESP32 RX <- EC200U TX (manual says 16; measured 12)
#define MODEM_TX_PIN 13  // ESP32 TX -> EC200U RX (manual says 17; measured 13)
#define MODEM_BAUD 115200
#define MODEM_RESET_PIN 9  // HIGH pulse resets the EC200U (found with Reset_Probe)
#define RESET_PULSE_MS 300

// Internet check: make sure the data context has an IP, then fetch the time
// from an NTP server. A blank APN works on O2 UK; set your operator's otherwise.
#define DATA_APN ""
#define NTP_SERVER "pool.ntp.org"
#define DATA_CHECK_INTERVAL_MS (2UL * 60 * 1000)
#define DATA_RETRY_MS (30UL * 1000)

#define CELL_SCAN_INTERVAL_MS 5000
#define NETWORK_SCAN_INTERVAL_MS (3UL * 60 * 1000)  // AT+QOPS takes ~40 s
#define NETWORK_RETRY_MS (60UL * 1000)
#define WDT_TIMEOUT_MS 30000
#define MAX_AT_TIMEOUTS 3  // consecutive timeouts before re-initialising the modem

#define TX_POWER_DBM 20            // ESP32-S3 maximum
#define SNAPSHOT_INTERVAL_MS 1000  // how often the advertised snapshot is rebuilt
#define ADV_INTERVAL_UNITS 160     // advertise every 100 ms (units of 0.625 ms)

#define COMPANY_ID 0xFFFF  // Bluetooth SIG ID reserved for testing
#define MAGIC0 '7'
#define MAGIC1 'S'
// v1 was one 31-byte legacy advert per record; v2 the snapshot; v3 adds the
// mobile data byte counters and the last NTP time to the status block.
#define PROTOCOL_VERSION 3
#define SNAP_MAX_CELLS 8
#define SNAP_MAX_NETS 6
#define SNAP_MAX_BYTES 240  // 36 header+status, 1+9+8*9 cells, 1+6*19 networks = 233

#define MODEM_STARTING 0
#define MODEM_NO_SIM 1
#define MODEM_SEARCHING 2
#define MODEM_REGISTERED 3
#define MODEM_NOT_RESPONDING 4

#define STATUS_FLAG_NET_SCAN 0x01  // AT+QOPS in progress

#define DATA_UNKNOWN 0     // not checked yet
#define DATA_OK 1          // NTP reply received over mobile data
#define DATA_NO_CONTEXT 2  // couldn't get an IP (APN / activation failed)
#define DATA_NTP_FAILED 3  // had an IP but the NTP request failed

#define NET_FLAG_MNC3 0x04
#define NET_FLAG_4G 0x10
#define NET_FLAG_3G 0x20
#define NET_FLAG_2G 0x40

#define KIND_SERVING 0
#define KIND_INTRA 1  // neighbour on the same frequency
#define KIND_INTER 2  // neighbour on another frequency

#define SINR_UNKNOWN -128

#define MAX_CELLS 12
#define MAX_NETWORKS 8
#define NET_NAME_LEN 9  // keeps the snapshot inside one extended advert PDU

struct Cell {
  uint16_t mcc, mnc;
  bool mnc3, tdd;
  uint8_t kind, band;
  uint16_t earfcn, pci;
  int8_t rsrp, rsrq, sinr;
  uint32_t cellId;
};

// One operator from AT+QOPS, summarised to its strongest cells.
struct Network {
  uint16_t mcc, mnc;
  uint8_t flags;          // NET_FLAG_*
  int8_t rsrp4g;          // strongest 4G cell RSRP, 0 = none
  uint8_t band4g;         // band of that cell
  uint8_t cells4g;        // distinct 4G cells seen
  int8_t rssi2g;          // strongest 2G cell RSSI, 0 = none
  char name[NET_NAME_LEN + 1];
};

static Cell cells[MAX_CELLS];
static int numCells = 0;
static Network nets[MAX_NETWORKS];
static int numNets = 0;
static SemaphoreHandle_t dataLock;

// Modem status, written by the modem task and read when building adverts.
static volatile uint8_t modemState = MODEM_STARTING;
static volatile uint8_t csq = 99;  // 99 = unknown
static volatile bool netScanRunning = false;
static volatile unsigned long lastCellOk = 0, lastNetOk = 0;  // millis(), 0 = never
static volatile uint8_t modemRecoveries = 0;  // soft re-initialisations
static volatile uint8_t modemHardResets = 0;  // GPIO9 resets
static volatile uint8_t dataState = DATA_UNKNOWN;
static volatile unsigned long lastDataOk = 0;  // millis(), 0 = never
static volatile uint16_t dataLatencyMs = 0;    // NTP round trip of the last good check
static volatile uint32_t dataBytesSent = 0;      // modem's mobile data counter (AT+QGDCNT)
static volatile uint32_t dataBytesReceived = 0;
static volatile uint32_t ntpEpoch = 0;  // UTC time from the last good NTP check, 0 = none

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

// The EC200U reports SINR as X where dB = X / 2 - 23.5 (AT manual, AT+QCSQ;
// AT+QENG's raw values track AT+QCSQ's, so the same scale is assumed).
int8_t sinrToDb(long raw) {
  return clamp8(lround(raw / 2.0 - 23.5));
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
    c.sinr = sinrToDb(f[i + 14].toInt());
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
      // The field after RSSI looks like SINR but stays fixed per cell while
      // RSRP moves, so it isn't a live reading; report SINR as unknown.
      c.sinr = SINR_UNKNOWN;
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
  xSemaphoreGive(dataLock);
  if (n > 0) lastCellOk = millis();

  for (int k = 0; k < n; k++) {
    const Cell &c = found[k];
    Serial.printf("%s %u-%0*u B%u EARFCN %u PCI %u RSRP %d RSRQ %d SINR %d\n",
                  c.kind == KIND_SERVING ? "SERV " : c.kind == KIND_INTRA ? "INTRA" : "INTER",
                  c.mcc, c.mnc3 ? 3 : 2, c.mnc, c.band, c.earfcn, c.pci, c.rsrp, c.rsrq, c.sinr);
  }
}

// Scans every operator in range with AT+QOPS (~40 s) and keeps, per operator,
// its strongest 4G cell, how many distinct 4G cells it has, and its strongest
// 2G cell. Output, one header per operator followed by its cells:
//   +QOPS: "O2 - UK","O2 - UK","23410"
//   1,"2G",120,5564,9727,25,-55,50,-,1          <arfcn>,<lac>,<ci>,<bsic>,<rssi>,...
//   6,"4G",347,215,8650,7B04573,-90,-14,-       <earfcn>,<pci>,<tac>,<ci>,<rsrp>,<rsrq>,...
bool scanNetworks() {
  Serial.println("Scanning networks (AT+QOPS), this takes about 40 s...");
  String resp = sendAT("AT+QOPS", 180000);
  if (resp.indexOf("+QOPS:") < 0) {
    Serial.println("QOPS scan failed: " + resp);
    return false;
  }

  Network found[MAX_NETWORKS];
  uint32_t seen4g[MAX_NETWORKS][16];  // earfcn << 16 | pci, to count distinct cells
  int n = 0;
  Network *cur = nullptr;

  forEachLine(resp, "", [&](const String &line) {
    if (line.startsWith("+QOPS:")) {
      cur = nullptr;
      String f[3];
      if (splitCsv(line.substring(6), f, 3) < 3 || f[2].length() < 5 || n >= MAX_NETWORKS) return;
      cur = &found[n++];
      memset(cur, 0, sizeof(Network));
      cur->mcc = f[2].substring(0, 3).toInt();
      cur->mnc = f[2].substring(3).toInt();
      if (f[2].length() >= 6) cur->flags |= NET_FLAG_MNC3;
      // Prefer the short name; some operators report none, so fall back to MCC-MNC.
      String name = f[1].length() ? f[1] : f[0].length() ? f[0] : f[2].substring(0, 3) + "-" + f[2].substring(3);
      strncpy(cur->name, name.c_str(), NET_NAME_LEN);
      cur->name[NET_NAME_LEN] = '\0';
      return;
    }
    if (!cur || line.length() == 0 || !isDigit(line[0])) return;

    String f[10];
    int nf = splitCsv(line, f, 10);
    if (nf < 7) return;
    if (f[1] == "4G" && nf >= 8) {
      long rsrp = f[6].toInt();
      if (rsrp < -140 || rsrp > -44) return;
      uint32_t key = ((uint32_t)f[2].toInt() << 16) | (f[3].toInt() & 0xFFFF);
      uint32_t *keys = seen4g[cur - found];
      bool dup = false;
      for (int k = 0; k < cur->cells4g; k++) dup |= keys[k] == key;
      if (!dup && cur->cells4g < 16) keys[cur->cells4g++] = key;
      cur->flags |= NET_FLAG_4G;
      if (cur->rsrp4g == 0 || rsrp > cur->rsrp4g) {
        cur->rsrp4g = rsrp;
        cur->band4g = earfcnToBand(f[2].toInt());
      }
    } else if (f[1] == "3G") {
      cur->flags |= NET_FLAG_3G;
    } else if (f[1] == "2G") {
      long rssi = f[6].toInt();
      cur->flags |= NET_FLAG_2G;
      if (rssi < -120 || rssi > -20) return;
      if (cur->rssi2g == 0 || rssi > cur->rssi2g) cur->rssi2g = rssi;
    }
  });

  xSemaphoreTake(dataLock, portMAX_DELAY);
  memcpy(nets, found, sizeof(Network) * n);
  numNets = n;
  xSemaphoreGive(dataLock);
  if (n > 0) lastNetOk = millis();

  for (int k = 0; k < n; k++) {
    const Network &nw = found[k];
    Serial.printf("NET %-9s %u-%0*u 4G: %d dBm B%u (%u cells)  2G: %d dBm\n", nw.name, nw.mcc,
                  (nw.flags & NET_FLAG_MNC3) ? 3 : 2, nw.mnc, nw.rsrp4g, nw.band4g, nw.cells4g, nw.rssi2g);
  }
  return n > 0;
}

// Reads modem output until a line starting with prefix arrives and returns
// it, or "" on timeout. Used for results that come after the OK, as URCs.
String waitForUrc(const char *prefix, unsigned long timeoutMs) {
  String line = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c != '\n') {
        line += c;
        continue;
      }
      line.trim();
      if (line.startsWith(prefix)) return line;
      if (line == "RDY") modemRebooted = true;
      line = "";
    }
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  return "";
}

// Seconds since 1970-01-01 UTC for a civil date and time (days-from-civil
// algorithm, valid for any Gregorian date).
uint32_t utcEpoch(int y, int mon, int d, int h, int minute, int s) {
  y -= mon <= 2;
  int era = y / 400;
  int yoe = y - era * 400;
  int doy = (153 * (mon + (mon > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468;
  return (uint32_t)(days * 86400L + h * 3600L + minute * 60L + s);
}

// Proves the modem can reach the internet: activates the data context if it
// has no IP (e.g. after a modem reset), then asks an NTP server for the time.
bool checkData() {
  String r = sendAT("AT+QIACT?");  // +QIACT: 1,1,1,"10.161.206.72" when active
  if (r.indexOf("+QIACT: 1,1") < 0) {
    Serial.println("Activating mobile data");
    sendAT("AT+QICSGP=1,1,\"" DATA_APN "\",\"\",\"\",0");
    r = sendAT("AT+QIACT=1", 150000);
    if (r.indexOf("OK") < 0) {
      dataState = DATA_NO_CONTEXT;
      Serial.println("Data activation failed: " + r);
      return false;
    }
  }

  // +QNTP: <err>,"<time>" follows the OK; err 0 = success.
  unsigned long t0 = millis();
  r = sendAT("AT+QNTP=1,\"" NTP_SERVER "\",123", 5000);
  String urc = "";
  int p = r.indexOf("+QNTP:");
  if (p >= 0) urc = r.substring(p);
  else if (r.indexOf("OK") >= 0) urc = waitForUrc("+QNTP:", 125000);  // AT manual max response time
  unsigned long latency = millis() - t0;

  if (urc.length() == 0 || urc.substring(6).toInt() != 0) {
    dataState = DATA_NTP_FAILED;
    Serial.println("NTP check failed: " + (urc.length() ? urc : r));
    return false;
  }
  dataState = DATA_OK;
  lastDataOk = millis();
  dataLatencyMs = (uint16_t)min(latency, 65535UL);
  // +QNTP: 0,"2026/10/10,21:19:54+22": the time is UTC (the modem's clock,
  // AT+CCLK, matched the PC's UTC); the +22 suffix is a quarter-hour zone.
  int Y, M, D, h, m, s;
  if (sscanf(urc.c_str(), "+QNTP: %*d,\"%d/%d/%d,%d:%d:%d", &Y, &M, &D, &h, &m, &s) == 6) {
    ntpEpoch = utcEpoch(Y, M, D, h, m, s);
  }
  Serial.printf("Internet OK: %s (%lu ms), data sent %lu B, received %lu B\n", urc.c_str(), latency,
                (unsigned long)dataBytesSent, (unsigned long)dataBytesReceived);
  return true;
}

void clearCells() {
  xSemaphoreTake(dataLock, portMAX_DELAY);
  numCells = 0;
  xSemaphoreGive(dataLock);
}

// Pulses the modem's hardware reset line, then waits for the RDY it prints
// ~4 s later. The modem can answer AT slightly before RDY, so without this
// wait the late RDY would look like a second, unexpected reboot.
void modemHardReset() {
  Serial.println("Hard-resetting modem (GPIO9)");
  digitalWrite(MODEM_RESET_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(RESET_PULSE_MS));
  digitalWrite(MODEM_RESET_PIN, LOW);
  modemHardResets++;

  modemRebooted = false;
  unsigned long start = millis();
  while (!modemRebooted && millis() - start < 15000) drainModem(100);
  Serial.println(modemRebooted ? "Modem sent RDY" : "No RDY after reset");
  modemRebooted = false;
}

// Waits for the modem to answer AT (it takes ~10 s after power-up), then
// applies the settings a modem reboot would have reset. If it never answers,
// hard-resets it and waits once more.
bool modemInit() {
  modemState = MODEM_STARTING;
  Serial.println("Waiting for modem...");
  bool alive = false;
  for (int attempt = 0; attempt < 2 && !alive; attempt++) {
    if (attempt == 1) {
      modemState = MODEM_NOT_RESPONDING;
      modemHardReset();
    }
    for (int i = 0; i < 20 && !alive; i++) {
      alive = sendAT("AT", 1000).indexOf("OK") >= 0;
    }
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

  // Mobile data byte counter: +QGDCNT: <bytes sent>,<bytes received>
  r = sendAT("AT+QGDCNT?");
  p = r.indexOf("+QGDCNT:");
  if (p >= 0) {
    String counts = r.substring(p + 8);
    int comma = counts.indexOf(',');
    if (comma > 0) {
      dataBytesSent = strtoul(counts.c_str(), nullptr, 10);
      dataBytesReceived = strtoul(counts.c_str() + comma + 1, nullptr, 10);
    }
  }
  return registered;
}

void modemTask(void *) {
  esp_task_wdt_add(NULL);
  bool ready = false;
  bool haveNets = false;
  unsigned long lastNetAttempt = 0;
  bool dataOk = false;
  unsigned long lastDataCheck = 0;

  for (;;) {
    esp_task_wdt_reset();

    if (!ready) {
      clearCells();
      lastDataCheck = 0;  // a reset modem loses its data context; re-check soon
      ready = modemInit();
      if (!ready) {
        Serial.println("Modem not responding, retrying in 5 s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    if (updateStatus()) {
      scanCells();
      unsigned long wait = haveNets ? NETWORK_SCAN_INTERVAL_MS : NETWORK_RETRY_MS;
      if (lastNetAttempt == 0 || millis() - lastNetAttempt > wait) {
        netScanRunning = true;
        haveNets = scanNetworks();
        netScanRunning = false;
        lastNetAttempt = millis();
      }
      unsigned long dataWait = dataOk ? DATA_CHECK_INTERVAL_MS : DATA_RETRY_MS;
      if (lastDataCheck == 0 || millis() - lastDataCheck > dataWait) {
        dataOk = checkData();
        lastDataCheck = millis();
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

void put16(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = v >> 8;
}

void put32(uint8_t *p, uint32_t v) {
  for (int b = 0; b < 4; b++) p[b] = (v >> (8 * b)) & 0xFF;
}

uint16_t ageSeconds(unsigned long t, unsigned long now) {
  if (t == 0) return 0xFFFF;  // never
  return (uint16_t)min((now - t) / 1000, 0xFFFEUL);
}

// Builds one snapshot (status, cells, networks) in the protocol v2 layout
// described in README.md. Returns its length, at most ~221 bytes.
int buildSnapshot(uint8_t *p) {
  static uint8_t snapshotSeq = 0;
  unsigned long now = millis();
  int i = 0;

  put16(p + i, COMPANY_ID); i += 2;
  p[i++] = MAGIC0;
  p[i++] = MAGIC1;
  p[i++] = PROTOCOL_VERSION;
  p[i++] = snapshotSeq++;

  // Status (18 bytes).
  p[i++] = modemState;
  p[i++] = netScanRunning ? STATUS_FLAG_NET_SCAN : 0;
  p[i++] = csq;
  put16(p + i, ageSeconds(lastCellOk, now)); i += 2;
  put16(p + i, ageSeconds(lastNetOk, now)); i += 2;
  uint32_t uptime = now / 1000;
  for (int b = 0; b < 4; b++) p[i++] = (uptime >> (8 * b)) & 0xFF;
  p[i++] = modemRecoveries;
  p[i++] = modemHardResets;
  p[i++] = dataState;
  put16(p + i, ageSeconds(lastDataOk, now)); i += 2;
  put16(p + i, dataLatencyMs); i += 2;
  put32(p + i, dataBytesSent); i += 4;
  put32(p + i, dataBytesReceived); i += 4;
  put32(p + i, ntpEpoch); i += 4;

  xSemaphoreTake(dataLock, portMAX_DELAY);

  // Cells: all share the serving operator, so MCC/MNC and the serving cell
  // ID are sent once. Serving cell first, then the strongest neighbours.
  Cell sorted[MAX_CELLS];
  memcpy(sorted, cells, sizeof(Cell) * numCells);
  std::sort(sorted, sorted + numCells, [](const Cell &a, const Cell &b) {
    if ((a.kind == KIND_SERVING) != (b.kind == KIND_SERVING)) return a.kind == KIND_SERVING;
    return a.rsrp > b.rsrp;
  });
  int nc = min(numCells, SNAP_MAX_CELLS);
  p[i++] = nc;
  if (nc > 0) {
    put16(p + i, sorted[0].mcc); i += 2;
    put16(p + i, sorted[0].mnc); i += 2;
    p[i++] = sorted[0].mnc3 ? NET_FLAG_MNC3 : 0;
    uint32_t servingId = sorted[0].kind == KIND_SERVING ? sorted[0].cellId : 0;
    for (int b = 0; b < 4; b++) p[i++] = (servingId >> (8 * b)) & 0xFF;
    for (int k = 0; k < nc; k++) {
      const Cell &c = sorted[k];
      p[i++] = (c.kind & 0x03) | (c.tdd ? 0x08 : 0);
      p[i++] = c.band;
      put16(p + i, c.earfcn); i += 2;
      put16(p + i, c.pci); i += 2;
      p[i++] = (uint8_t)c.rsrp;
      p[i++] = (uint8_t)c.rsrq;
      p[i++] = (uint8_t)c.sinr;
    }
  }

  // Networks.
  int nn = min(numNets, SNAP_MAX_NETS);
  p[i++] = nn;
  for (int k = 0; k < nn; k++) {
    const Network &nw = nets[k];
    put16(p + i, nw.mcc); i += 2;
    put16(p + i, nw.mnc); i += 2;
    p[i++] = nw.flags;
    p[i++] = (uint8_t)nw.rsrp4g;
    p[i++] = nw.band4g;
    p[i++] = nw.cells4g;
    p[i++] = (uint8_t)nw.rssi2g;
    int len = strlen(nw.name);
    p[i++] = len;
    memcpy(p + i, nw.name, len);
    i += len;
  }

  xSemaphoreGive(dataLock);
  return i;
}

// Replaces the advertised snapshot. Restarting the instance keeps the update
// atomic; the gap is a few milliseconds.
void advertise(const uint8_t *data, int len) {
  NimBLEExtAdvertisement ad(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_1M);
  ad.setLegacyAdvertising(false);  // BLE 5 extended advert: room for the whole snapshot
  ad.setConnectable(false);
  ad.setScannable(false);
  ad.setTxPower(TX_POWER_DBM);
  ad.setMinInterval(ADV_INTERVAL_UNITS);
  ad.setMaxInterval(ADV_INTERVAL_UNITS);
  ad.setManufacturerData(data, len);

  NimBLEExtAdvertising *ext = NimBLEDevice::getAdvertising();
  ext->stop(0);
  if (!ext->setInstanceData(0, ad) || !ext->start(0)) {
    Serial.println("Extended advertising update failed");
  }
  // NimBLEDevice::setPower() covers legacy states; an extended advertising
  // set has its own power, so set it on instance 0 directly.
  esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_ADV, 0, ESP_PWR_LVL_P20);
}

// TX power of advertising instance 0 in dBm, as the controller reports it.
int advertPowerDbm() {
  int level = esp_ble_tx_power_get_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_ADV, 0);
  if (level < 0) return -128;
  return min((level - ESP_PWR_LVL_N0) * 3, 20);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(2000);
  Serial.println("7Semi ESP32-S3 EC200U - LTE Scanner Beacon");

  dataLock = xSemaphoreCreateMutex();

  NimBLEDevice::init("7Semi-LTE");
  NimBLEDevice::setPower(TX_POWER_DBM);

  // Restart the ESP32 if the modem task or this loop stops checking in.
  esp_task_wdt_config_t wdt = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);  // setup() and loop() run in the same task

  // Hold the reset line released (LOW) rather than leaving it floating.
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, LOW);

  // All modem I/O happens in its own task, including start-up.
  xTaskCreatePinnedToCore(modemTask, "modem", 12288, nullptr, 1, nullptr, 1);
}

void loop() {
  // A fresh snapshot every second: data ages stay current, and any change
  // reaches the display within a second.
  static unsigned long lastLog = 0;
  uint8_t snap[SNAP_MAX_BYTES];
  int len = buildSnapshot(snap);
  advertise(snap, len);
  if (lastLog == 0 || millis() - lastLog >= 60000) {
    lastLog = millis();
    Serial.printf("BLE: snapshot %d bytes, advert TX power %d dBm\n", len, advertPowerDbm());
  }
  esp_task_wdt_reset();
  delay(SNAPSHOT_INTERVAL_MS);
}
