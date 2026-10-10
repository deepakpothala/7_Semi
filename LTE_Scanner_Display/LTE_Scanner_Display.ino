// LilyGO T-Display-S3: receiver for LTE_Scanner_Beacon.
// Passively scans for the 7Semi beacon's BLE 5 extended adverts. Each one is
// a complete snapshot (modem status, cells, networks) that replaces what is
// shown, and the dashboard on the 320x170 display shows the serving cell
// with signal bars, neighbour cells, and visible networks. The status part
// gives the modem state and how old the cell data is, so stale or missing
// data is shown as such rather than as live.
//
// Built for Arduino-ESP32 core 3.x. Needs the LovyanGFX library and the
// NimBLE-Arduino library (2.x) with CONFIG_BT_NIMBLE_EXT_ADV enabled in its
// nimconfig.h, so it can receive extended adverts.
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled, PSRAM: OPI PSRAM.
// Advert format: see README.md "BLE advert format".

#include <LovyanGFX.hpp>
#include <NimBLEDevice.h>
#include <esp_task_wdt.h>
#include <algorithm>

#if !CONFIG_BT_NIMBLE_EXT_ADV
#error "Enable CONFIG_BT_NIMBLE_EXT_ADV in NimBLE-Arduino's src/nimconfig.h"
#endif

#define PIN_LCD_POWER 15

#define COMPANY_ID 0xFFFF
#define MAGIC0 '7'
#define MAGIC1 'S'
#define PROTOCOL_VERSION 2

#define NET_FLAG_4G 0x10
#define NET_FLAG_3G 0x20
#define NET_FLAG_2G 0x40

#define MODEM_STARTING 0
#define MODEM_NO_SIM 1
#define MODEM_SEARCHING 2
#define MODEM_REGISTERED 3
#define MODEM_NOT_RESPONDING 4
#define STATUS_FLAG_NET_SCAN 0x01  // AT+QOPS in progress

#define DATA_UNKNOWN 0
#define DATA_OK 1
#define DATA_NO_CONTEXT 2
#define DATA_NTP_FAILED 3
#define DATA_STALE_S 300  // beacon checks every 2 min; older than this = not proven

#define KIND_SERVING 0
#define KIND_INTRA 1
#define KIND_INTER 2

#define MAX_CELLS 16
#define MAX_NETWORKS 8
#define EXPIRE_MS 30000  // clear everything if no snapshot arrives for this long
#define STALE_MS 5000    // beacon counts as lost after this long without adverts
#define CELL_STALE_S 15  // serving cell readout dims when older than this
#define REDRAW_MS 500
#define WDT_TIMEOUT_MS 15000

// T-Display-S3: ST7789 170x320 on an 8-bit parallel bus.
class LGFX_TDisplayS3 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_Parallel8 _bus;
  lgfx::Light_PWM _light;

 public:
  LGFX_TDisplayS3() {
    {
      auto cfg = _bus.config();
      cfg.freq_write = 20000000;
      cfg.pin_wr = 8;
      cfg.pin_rd = 9;
      cfg.pin_rs = 7;  // D/C
      cfg.pin_d0 = 39;
      cfg.pin_d1 = 40;
      cfg.pin_d2 = 41;
      cfg.pin_d3 = 42;
      cfg.pin_d4 = 45;
      cfg.pin_d5 = 46;
      cfg.pin_d6 = 47;
      cfg.pin_d7 = 48;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = 6;
      cfg.pin_rst = 5;
      cfg.pin_busy = -1;
      cfg.panel_width = 170;
      cfg.panel_height = 320;
      cfg.offset_x = 35;
      cfg.offset_y = 0;
      cfg.readable = false;
      cfg.invert = true;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl = 38;
      cfg.freq = 22000;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static LGFX_TDisplayS3 tft;
static LGFX_Sprite canvas(&tft);  // draw off-screen, push once: no flicker

#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
const uint16_t C_BG = RGB565(9, 13, 22);
const uint16_t C_CARD = RGB565(22, 29, 45);
const uint16_t C_CHIP = RGB565(36, 46, 68);
const uint16_t C_TEXT = RGB565(235, 240, 248);
const uint16_t C_DIM = RGB565(125, 138, 162);
const uint16_t C_FAINT = RGB565(52, 63, 88);
const uint16_t C_ACCENT = RGB565(70, 160, 255);
const uint16_t C_GREEN = RGB565(52, 211, 120);
const uint16_t C_LIME = RGB565(150, 215, 60);
const uint16_t C_AMBER = RGB565(250, 190, 40);
const uint16_t C_ORANGE = RGB565(250, 130, 40);
const uint16_t C_RED = RGB565(240, 72, 72);

struct CellRx {
  bool used;
  uint8_t kind, band;
  uint16_t mcc, mnc;
  bool mnc3, tdd;
  uint16_t earfcn, pci;
  int8_t rsrp, rsrq, sinr;
  uint32_t cellId;
  unsigned long seen;
};

// One operator from the beacon's AT+QOPS scan, summarised to its best cells.
struct NetRx {
  bool used;
  uint16_t mcc, mnc;
  uint8_t flags;    // NET_FLAG_*
  int8_t rsrp4g;    // strongest 4G cell RSRP, 0 = none
  uint8_t band4g;
  uint8_t cells4g;  // distinct 4G cells
  int8_t rssi2g;    // strongest 2G cell RSSI, 0 = none
  char name[10];
  unsigned long seen;
};

// One row of the networks panel.
struct Network {
  NetRx net;
  bool current;  // the network the beacon's SIM is registered on
};

struct Quality {
  const char *label;
  uint8_t bars;
  uint16_t color;
};

struct StatusRx {
  bool valid;
  uint8_t state, flags, csq, recoveries, hardResets;
  uint16_t cellAge, netAge;  // seconds when sent, 0xFFFF = never
  uint32_t uptime;
  uint8_t dataState;         // DATA_*
  uint16_t dataAge;          // seconds since the last good internet check
  uint16_t dataLatencyMs;    // NTP round trip of that check
  unsigned long seen;
};

static CellRx cells[MAX_CELLS];
static NetRx netTable[MAX_NETWORKS];
static StatusRx beaconStatus;
static int beaconRssi = 0;
static unsigned long lastPacket = 0;
static uint32_t snapshotsReceived = 0;  // for the serial log
static size_t lastSnapshotLen = 0;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- BLE ----------

uint16_t get16(const uint8_t *p) {
  return p[0] | (p[1] << 8);
}

uint32_t get32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Decodes one protocol v2 snapshot (see README.md) and, if it is well
// formed, replaces everything the display knows in one step. Cells that the
// beacon no longer reports therefore disappear with the next snapshot.
bool decodeSnapshot(const uint8_t *d, size_t n, int rssi) {
  if (n < 25 || get16(d) != COMPANY_ID || d[2] != MAGIC0 || d[3] != MAGIC1) return false;
  if (d[4] != PROTOCOL_VERSION) return false;
  unsigned long now = millis();
  size_t i = 6;

  StatusRx s;
  s.valid = true;
  s.state = d[i++];
  s.flags = d[i++];
  s.csq = d[i++];
  s.cellAge = get16(d + i); i += 2;
  s.netAge = get16(d + i); i += 2;
  s.uptime = get32(d + i); i += 4;
  s.recoveries = d[i++];
  s.hardResets = d[i++];
  s.dataState = d[i++];
  s.dataAge = get16(d + i); i += 2;
  s.dataLatencyMs = get16(d + i); i += 2;
  s.seen = now;

  CellRx c[MAX_CELLS] = {};
  int nc = d[i++];
  if (nc > MAX_CELLS) return false;
  if (nc > 0) {
    if (i + 9 + nc * 9 > n) return false;
    uint16_t mcc = get16(d + i), mnc = get16(d + i + 2);
    bool mnc3 = d[i + 4] & 0x04;
    uint32_t servingId = get32(d + i + 5);
    i += 9;
    for (int k = 0; k < nc; k++) {
      c[k].used = true;
      c[k].mcc = mcc;
      c[k].mnc = mnc;
      c[k].mnc3 = mnc3;
      c[k].kind = d[i] & 0x03;
      c[k].tdd = d[i] & 0x08;
      c[k].band = d[i + 1];
      c[k].earfcn = get16(d + i + 2);
      c[k].pci = get16(d + i + 4);
      c[k].rsrp = (int8_t)d[i + 6];
      c[k].rsrq = (int8_t)d[i + 7];
      c[k].sinr = (int8_t)d[i + 8];
      c[k].cellId = c[k].kind == KIND_SERVING ? servingId : 0;
      c[k].seen = now;
      i += 9;
    }
  }

  NetRx w[MAX_NETWORKS] = {};
  if (i >= n) return false;
  int nn = d[i++];
  if (nn > MAX_NETWORKS) return false;
  for (int k = 0; k < nn; k++) {
    if (i + 10 > n) return false;
    w[k].used = true;
    w[k].mcc = get16(d + i);
    w[k].mnc = get16(d + i + 2);
    w[k].flags = d[i + 4];
    w[k].rsrp4g = (int8_t)d[i + 5];
    w[k].band4g = d[i + 6];
    w[k].cells4g = d[i + 7];
    w[k].rssi2g = (int8_t)d[i + 8];
    size_t len = d[i + 9];
    i += 10;
    if (i + len > n) return false;
    size_t copy = min(len, sizeof(w[k].name) - 1);
    memcpy(w[k].name, d + i, copy);
    w[k].name[copy] = '\0';
    w[k].seen = now;
    i += len;
  }

  portENTER_CRITICAL(&dataMux);
  memcpy(cells, c, sizeof(cells));
  memcpy(netTable, w, sizeof(netTable));
  beaconStatus = s;
  beaconRssi = rssi;
  lastPacket = now;
  snapshotsReceived++;
  lastSnapshotLen = n;
  portEXIT_CRITICAL(&dataMux);
  return true;
}

class BeaconCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    if (!dev->haveManufacturerData()) return;
    std::string md = dev->getManufacturerData();
    decodeSnapshot((const uint8_t *)md.data(), md.size(), dev->getRSSI());
  }

  // The scan only stops if something goes wrong; start it again.
  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    NimBLEDevice::getScan()->start(0, false, true);
  }
};


// ---------- Drawing ----------

Quality rsrpQuality(int rsrp) {
  if (rsrp >= -80) return {"EXCELLENT", 5, C_GREEN};
  if (rsrp >= -90) return {"GOOD", 4, C_LIME};
  if (rsrp >= -100) return {"FAIR", 3, C_AMBER};
  if (rsrp >= -110) return {"POOR", 2, C_ORANGE};
  return {"WEAK", 1, C_RED};
}

// Copies s into out, trimmed with ".." until it fits maxW pixels in the current font.
void fitText(const char *s, char *out, size_t outSize, int maxW) {
  strncpy(out, s, outSize - 1);
  out[outSize - 1] = '\0';
  size_t len = strlen(out);
  while (len > 1 && canvas.textWidth(out) > maxW) {
    out[--len] = '\0';
    if (len > 2) {
      out[len - 1] = '.';
      out[len - 2] = '.';
    }
  }
}

void drawSignalBars(int x, int bottom, uint8_t bars, uint16_t color) {
  const int w = 7, gap = 4;
  for (int i = 0; i < 5; i++) {
    int h = 8 + i * 7;
    canvas.fillSmoothRoundRect(x + i * (w + gap), bottom - h, w, h, 2, i < bars ? color : C_FAINT);
  }
}

void drawPill(int x, int y, int w, int h, uint16_t bg, uint16_t fg, const char *text) {
  canvas.fillSmoothRoundRect(x, y, w, h, h / 2, bg);
  canvas.setTextColor(fg);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.drawString(text, x + w / 2, y + h / 2 + 1);
}

// Same hue at half brightness, for stale readings.
uint16_t dim(uint16_t c) {
  return (c >> 1) & 0x7BEF;
}

bool modemProblem(const StatusRx &st) {
  return st.valid && st.state != MODEM_REGISTERED;
}

void drawHeader(unsigned long now, unsigned long last, int rssi, const StatusRx &st) {
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(C_TEXT);
  canvas.setTextDatum(textdatum_t::middle_left);
  int titleEnd = 8 + canvas.drawString("LTE Scanner", 8, 12);

  // Green: all good. Amber: beacon heard but its modem isn't registered.
  // Red: beacon not heard.
  char status[32];
  uint16_t color;
  bool live = last && now - last < STALE_MS;
  if (last == 0) {
    snprintf(status, sizeof(status), "searching...");
    color = C_RED;
  } else if (!live) {
    snprintf(status, sizeof(status), "beacon lost %lus", (now - last) / 1000);
    color = C_RED;
  } else if (modemProblem(st)) {
    static const char *names[] = {"modem starting", "no SIM", "no network", "", "modem silent"};
    snprintf(status, sizeof(status), "%s", st.state < 5 ? names[st.state] : "modem ?");
    color = C_AMBER;
  } else {
    snprintf(status, sizeof(status), "BLE %d dBm", rssi);
    color = C_GREEN;
  }

  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(color == C_GREEN ? C_DIM : color);
  canvas.setTextDatum(textdatum_t::middle_right);
  int w = canvas.drawString(status, 312, 12);
  canvas.fillSmoothCircle(312 - w - 8, 12, 3, color);

  // Internet: proven by the beacon's last NTP check over mobile data.
  if (!live || !st.valid || st.dataState == DATA_UNKNOWN) return;
  char net[24];
  uint16_t netColor;
  long age = st.dataAge == 0xFFFF ? -1 : st.dataAge + (long)(now - st.seen) / 1000;
  if (st.dataState == DATA_OK && age >= 0 && age <= DATA_STALE_S) {
    snprintf(net, sizeof(net), "web %u ms", st.dataLatencyMs);
    netColor = C_GREEN;
  } else if (st.dataState == DATA_NO_CONTEXT) {
    snprintf(net, sizeof(net), "no data");
    netColor = C_AMBER;
  } else {
    snprintf(net, sizeof(net), "web down");
    netColor = C_AMBER;
  }
  // Between the title and the right-hand status; drop the text, keeping the
  // dot, if it would run into the status.
  int dotX = titleEnd + 12;
  int statusStart = 312 - w - 14;
  canvas.fillSmoothCircle(dotX, 12, 3, netColor);
  if (dotX + 8 + canvas.textWidth(net) < statusStart) {
    canvas.setTextColor(netColor == C_GREEN ? C_DIM : netColor);
    canvas.setTextDatum(textdatum_t::middle_left);
    canvas.drawString(net, dotX + 8, 12);
  }
}

void drawCardMessage(int x, int y, int w, int h, const char *title, const char *detail, uint16_t color) {
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(color);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.drawString(title, x + w / 2, y + h / 2 - 8);
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(C_DIM);
  canvas.drawString(detail, x + w / 2, y + h / 2 + 10);
}

// cellAge: seconds since the beacon last read the serving cell, -1 = unknown.
void drawServingCard(const CellRx *serving, const Network *nets, int numNets,
                     const StatusRx &st, long cellAge) {
  const int x = 6, y = 26, w = 198, h = 92;
  canvas.fillSmoothRoundRect(x, y, w, h, 8, C_CARD);

  if (modemProblem(st) && !serving) {
    char detail[32];
    switch (st.state) {
      case MODEM_STARTING:
        drawCardMessage(x, y, w, h, "Modem starting", "waiting for it to answer", C_AMBER);
        break;
      case MODEM_NO_SIM:
        drawCardMessage(x, y, w, h, "SIM not ready", "check the SIM card", C_AMBER);
        break;
      case MODEM_SEARCHING:
        snprintf(detail, sizeof(detail), "signal CSQ %u", st.csq);
        drawCardMessage(x, y, w, h, "Searching for network", st.csq == 99 ? "no signal" : detail, C_AMBER);
        break;
      default:
        snprintf(detail, sizeof(detail), "check power - restarts: %u", st.recoveries + st.hardResets);
        drawCardMessage(x, y, w, h, "Modem not responding", detail, C_RED);
        break;
    }
    return;
  }

  if (!serving) {
    drawCardMessage(x, y, w, h, lastPacket ? "No serving cell" : "Waiting for beacon",
                    lastPacket ? "modem has no LTE cell yet" : "is the transmitter on?", C_DIM);
    return;
  }

  bool stale = cellAge > CELL_STALE_S;
  Quality q = rsrpQuality(serving->rsrp);
  if (stale) q.color = dim(q.color);

  // Operator name from the networks list, else MCC-MNC.
  char opName[20];
  snprintf(opName, sizeof(opName), "%u-%0*u", serving->mcc, serving->mnc3 ? 3 : 2, serving->mnc);
  for (int i = 0; i < numNets; i++) {
    if (nets[i].net.mcc == serving->mcc && nets[i].net.mnc == serving->mnc) {
      strncpy(opName, nets[i].net.name, sizeof(opName) - 1);
      opName[sizeof(opName) - 1] = '\0';
    }
  }

  canvas.setFont(&fonts::DejaVu9);
  int pillW = canvas.textWidth(q.label) + 14;
  drawPill(x + w - pillW - 8, y + 7, pillW, 15, q.color, C_BG, q.label);

  canvas.setFont(&fonts::DejaVu12);
  char fitted[20];
  fitText(opName, fitted, sizeof(fitted), w - pillW - 28);
  canvas.setTextColor(C_TEXT);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString(fitted, x + 10, y + 8);

  // Large RSRP readout.
  char rsrp[8];
  snprintf(rsrp, sizeof(rsrp), "%d", serving->rsrp);
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextColor(stale ? C_DIM : C_TEXT);
  canvas.setTextDatum(textdatum_t::baseline_left);
  int nw = canvas.drawString(rsrp, x + 8, y + 64);
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(C_DIM);
  canvas.drawString("dBm", x + 8 + nw + 4, y + 64);

  drawSignalBars(x + w - 62, y + 64, q.bars, q.color);

  // Bottom line: cell details while fresh, a warning once the reading is old.
  // The modem can't read cells during its ~40 s network scan, so say so.
  char detail[48];
  if (stale && st.valid && (st.flags & STATUS_FLAG_NET_SCAN)) {
    snprintf(detail, sizeof(detail), "scanning networks... %lds", cellAge);
  } else if (stale) {
    snprintf(detail, sizeof(detail), "no update for %lds", cellAge);
  } else {
    snprintf(detail, sizeof(detail), "Band %u   PCI %u   RSRQ %d dB", serving->band, serving->pci, serving->rsrq);
  }
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(stale ? C_AMBER : C_DIM);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString(detail, x + 10, y + 74);
}

void drawNeighbourCard(const CellRx *list, int count) {
  const int x = 6, y = 124, w = 198, h = 40;
  canvas.fillSmoothRoundRect(x, y, w, h, 8, C_CARD);
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(C_DIM);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString("NEIGHBOUR CELLS", x + 10, y + 5);

  if (count == 0) {
    canvas.setTextColor(C_FAINT);
    canvas.drawString("none reported", x + 10, y + 22);
    return;
  }

  int cx = x + 8;
  for (int i = 0; i < count; i++) {
    char label[16];
    snprintf(label, sizeof(label), "B%u %d", list[i].band, list[i].rsrp);
    int cw = canvas.textWidth(label) + 22;
    if (cx + cw > x + w - 6) break;
    canvas.fillSmoothRoundRect(cx, y + 19, cw, 16, 8, C_CHIP);
    canvas.fillSmoothCircle(cx + 8, y + 27, 3, rsrpQuality(list[i].rsrp).color);
    canvas.setTextColor(C_TEXT);
    canvas.setTextDatum(textdatum_t::middle_left);
    canvas.drawString(label, cx + 15, y + 28);
    cx += cw + 5;
  }
}

void drawNetworksCard(const Network *nets, int count, bool scanning) {
  const int x = 210, y = 26, w = 104, h = 138;
  canvas.fillSmoothRoundRect(x, y, w, h, 8, C_CARD);
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(C_DIM);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString("NETWORKS", x + 9, y + 5);
  if (scanning && count > 0) {
    canvas.fillSmoothCircle(x + w - 10, y + 9, 3, C_AMBER);  // a fresh scan is running
  }

  if (count == 0) {
    canvas.setTextColor(C_FAINT);
    canvas.drawString("scanning...", x + 9, y + 24);
    return;
  }

  const int maxRows = 5;
  for (int i = 0; i < count && i < maxRows; i++) {
    int ry = y + 22 + i * 22;
    if (i == maxRows - 1 && count > maxRows) {
      char more[16];
      snprintf(more, sizeof(more), "+%d more", count - maxRows + 1);
      canvas.setFont(&fonts::DejaVu9);
      canvas.setTextColor(C_DIM);
      canvas.setTextDatum(textdatum_t::top_left);
      canvas.drawString(more, x + 9, ry + 2);
      break;
    }
    const NetRx &n = nets[i].net;
    bool current = nets[i].current;

    // Right: strongest 4G RSRP coloured by quality, or the 2G level if the
    // operator has no 4G here.
    char value[12];
    uint16_t valueColor;
    int barRsrp = -140;
    canvas.setTextDatum(textdatum_t::top_right);
    if (n.rsrp4g) {
      snprintf(value, sizeof(value), "%d", n.rsrp4g);
      valueColor = rsrpQuality(n.rsrp4g).color;
      barRsrp = n.rsrp4g;
      canvas.setFont(&fonts::DejaVu12);
    } else {
      snprintf(value, sizeof(value), n.rssi2g ? "2G %d" : "--", n.rssi2g);
      valueColor = C_DIM;
      canvas.setFont(&fonts::DejaVu9);
    }
    canvas.setTextColor(valueColor);
    int vw = canvas.drawString(value, x + w - 7, ry);

    canvas.fillSmoothCircle(x + 11, ry + 6, 3, current ? C_GREEN : C_FAINT);
    canvas.setFont(&fonts::DejaVu12);
    char fitted[12];
    fitText(n.name, fitted, sizeof(fitted), w - 30 - vw);
    canvas.setTextColor(current ? C_GREEN : C_TEXT);
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.drawString(fitted, x + 19, ry);

    // Thin signal bar under the name: RSRP -140 (empty) .. -44 dBm (full).
    const int bx = x + 19, bw = w - 26;
    canvas.fillRect(bx, ry + 15, bw, 3, C_FAINT);
    int fill = constrain((barRsrp + 140) * bw / 96, 0, bw);
    if (fill > 0) canvas.fillRect(bx, ry + 15, fill, 3, valueColor);
  }
}

void drawScreen() {
  CellRx c[MAX_CELLS];
  NetRx nr[MAX_NETWORKS];
  StatusRx st;
  int rssi;
  unsigned long last, now = millis();

  portENTER_CRITICAL(&dataMux);
  for (auto &x : cells) if (x.used && now - x.seen > EXPIRE_MS) x.used = false;
  for (auto &x : netTable) if (x.used && now - x.seen > EXPIRE_MS) x.used = false;
  if (beaconStatus.valid && now - beaconStatus.seen > EXPIRE_MS) beaconStatus.valid = false;
  memcpy(c, cells, sizeof(c));
  memcpy(nr, netTable, sizeof(nr));
  st = beaconStatus;
  rssi = beaconRssi;
  last = lastPacket;
  portEXIT_CRITICAL(&dataMux);

  // Age of the serving-cell reading: as reported by the beacon, plus the time
  // since that status advert arrived. Without status, fall back to how long
  // ago the cell advert itself was heard.
  long cellAge = -1;
  if (st.valid && st.cellAge != 0xFFFF) cellAge = st.cellAge + (now - st.seen) / 1000;

  // Serving cell, and neighbours sorted strongest first.
  const CellRx *serving = nullptr;
  CellRx neigh[MAX_CELLS];
  int numNeigh = 0;
  for (auto &x : c) {
    if (!x.used) continue;
    if (x.kind == KIND_SERVING) {
      if (!serving || x.seen > serving->seen) serving = &x;
    } else {
      neigh[numNeigh++] = x;
    }
  }
  std::sort(neigh, neigh + numNeigh, [](const CellRx &a, const CellRx &b) { return a.rsrp > b.rsrp; });

  // Networks: the one the SIM is registered on (matches the serving cell)
  // first, then strongest 4G signal first.
  Network nets[MAX_NETWORKS];
  int numNets = 0;
  for (auto &x : nr) {
    if (!x.used) continue;
    nets[numNets].net = x;
    nets[numNets].current = serving && serving->mcc == x.mcc && serving->mnc == x.mnc;
    numNets++;
  }
  std::sort(nets, nets + numNets, [](const Network &a, const Network &b) {
    if (a.current != b.current) return a.current;
    int ra = a.net.rsrp4g ? a.net.rsrp4g : -200, rb = b.net.rsrp4g ? b.net.rsrp4g : -200;
    return ra > rb;
  });

  if (serving && !st.valid) cellAge = (now - serving->seen) / 1000;

  canvas.fillScreen(C_BG);
  drawHeader(now, last, rssi, st);
  drawServingCard(serving, nets, numNets, st, cellAge);
  drawNeighbourCard(neigh, numNeigh);
  drawNetworksCard(nets, numNets, st.valid && (st.flags & STATUS_FLAG_NET_SCAN));
  canvas.pushSprite(0, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LCD_POWER, OUTPUT);
  digitalWrite(PIN_LCD_POWER, HIGH);
  tft.init();
  tft.setRotation(1);  // landscape, USB on the right; use 3 to flip
  tft.setBrightness(200);
  canvas.setColorDepth(16);
  if (!canvas.createSprite(tft.width(), tft.height())) Serial.println("Sprite alloc failed");
  drawScreen();

  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  // wantDuplicates = true: every snapshot comes from the same address.
  scan->setScanCallbacks(new BeaconCallbacks(), true);
  scan->setDuplicateFilter(false);
  scan->setMaxResults(0);  // decode in the callback; keep nothing in memory
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(100);  // window == interval: listen continuously
  scan->start(0, false, true);  // 0 = scan forever, in the background
  Serial.println("Scanning for 7Semi LTE beacon...");

  // Restart if the draw loop ever stops running.
  esp_task_wdt_config_t wdt = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);
}

void loop() {
  static unsigned long lastDraw = 0, lastLog = 0;
  if (millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    drawScreen();
    esp_task_wdt_reset();
  }
  if (millis() - lastLog >= 10000) {
    lastLog = millis();
    Serial.printf("snapshots: %lu, last %u bytes, beacon RSSI %d dBm\n",
                  (unsigned long)snapshotsReceived, (unsigned)lastSnapshotLen, beaconRssi);
  }
}
