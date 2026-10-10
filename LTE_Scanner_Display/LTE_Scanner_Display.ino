// LilyGO T-Display-S3: receiver for LTE_Scanner_Beacon.
// Passively scans BLE for the 7Semi beacon's adverts, collects the operator
// and LTE cell records, and draws a dashboard on the 320x170 display:
// serving cell with signal bars, neighbour cells, and visible networks.
//
// Built for Arduino-ESP32 core 3.x. Needs the LovyanGFX library.
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled, PSRAM: OPI PSRAM.
// Advert format: see README.md "BLE advert format".

#include <LovyanGFX.hpp>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define PIN_LCD_POWER 15

#define MAGIC0 '7'
#define MAGIC1 'S'
#define TYPE_CELL 1
#define TYPE_OPERATOR 2
#define KIND_SERVING 0
#define KIND_INTRA 1
#define KIND_INTER 2

#define MAX_CELLS 16
#define MAX_OPERATORS 8
#define EXPIRE_MS 30000  // drop records the beacon stopped sending
#define STALE_MS 5000    // beacon counts as lost after this long without adverts
#define REDRAW_MS 500

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

struct OpRx {
  bool used;
  uint16_t mcc, mnc;
  bool mnc3;
  uint8_t stat, act;
  char name[12];
  unsigned long seen;
};

// One row of the networks panel: an operator with all its technologies merged.
struct Network {
  uint16_t mcc, mnc;
  char name[12];
  bool current, has4g, has3g, has2g;
};

struct Quality {
  const char *label;
  uint8_t bars;
  uint16_t color;
};

static CellRx cells[MAX_CELLS];
static OpRx ops[MAX_OPERATORS];
static int beaconRssi = 0;
static unsigned long lastPacket = 0;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- BLE ----------

uint16_t get16(const uint8_t *p) {
  return p[0] | (p[1] << 8);
}

// Slot for this cell: the existing entry, else a free one, else the oldest.
int cellSlot(uint16_t earfcn, uint16_t pci) {
  int oldest = 0;
  for (int i = 0; i < MAX_CELLS; i++) {
    if (cells[i].used && cells[i].earfcn == earfcn && cells[i].pci == pci) return i;
  }
  for (int i = 0; i < MAX_CELLS; i++) {
    if (!cells[i].used) return i;
    if (cells[i].seen < cells[oldest].seen) oldest = i;
  }
  return oldest;
}

int opSlot(uint16_t mcc, uint16_t mnc, uint8_t act) {
  int oldest = 0;
  for (int i = 0; i < MAX_OPERATORS; i++) {
    if (ops[i].used && ops[i].mcc == mcc && ops[i].mnc == mnc && ops[i].act == act) return i;
  }
  for (int i = 0; i < MAX_OPERATORS; i++) {
    if (!ops[i].used) return i;
    if (ops[i].seen < ops[oldest].seen) oldest = i;
  }
  return oldest;
}

class BeaconCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveManufacturerData()) return;
    String md = dev.getManufacturerData();
    const uint8_t *d = (const uint8_t *)md.c_str();
    size_t n = md.length();
    if (n < 8 || d[0] != 0xFF || d[1] != 0xFF || d[2] != MAGIC0 || d[3] != MAGIC1) return;

    unsigned long now = millis();
    uint8_t type = d[4];

    if (type == TYPE_CELL && n >= 25) {
      CellRx c;
      c.used = true;
      c.mcc = get16(d + 8);
      c.mnc = get16(d + 10);
      c.kind = d[12] & 0x03;
      c.mnc3 = d[12] & 0x04;
      c.tdd = d[12] & 0x08;
      c.band = d[13];
      c.earfcn = get16(d + 14);
      c.pci = get16(d + 16);
      c.rsrp = (int8_t)d[18];
      c.rsrq = (int8_t)d[19];
      c.sinr = (int8_t)d[20];
      c.cellId = d[21] | (d[22] << 8) | (d[23] << 16) | ((uint32_t)d[24] << 24);
      c.seen = now;
      portENTER_CRITICAL(&dataMux);
      cells[cellSlot(c.earfcn, c.pci)] = c;
      beaconRssi = dev.getRSSI();
      lastPacket = now;
      portEXIT_CRITICAL(&dataMux);
    } else if (type == TYPE_OPERATOR && n >= 15) {
      OpRx o;
      o.used = true;
      o.mcc = get16(d + 8);
      o.mnc = get16(d + 10);
      o.mnc3 = d[12] & 0x04;
      o.stat = d[13];
      o.act = d[14];
      size_t len = min(n - 15, sizeof(o.name) - 1);
      memcpy(o.name, d + 15, len);
      o.name[len] = '\0';
      o.seen = now;
      portENTER_CRITICAL(&dataMux);
      ops[opSlot(o.mcc, o.mnc, o.act)] = o;
      beaconRssi = dev.getRSSI();
      lastPacket = now;
      portEXIT_CRITICAL(&dataMux);
    }
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

void drawHeader(unsigned long now, unsigned long last, int rssi) {
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(C_TEXT);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.drawString("LTE Scanner", 8, 12);

  char status[32];
  bool live = last && now - last < STALE_MS;
  if (last == 0) snprintf(status, sizeof(status), "searching...");
  else if (live) snprintf(status, sizeof(status), "BLE %d dBm", rssi);
  else snprintf(status, sizeof(status), "lost %lus ago", (now - last) / 1000);

  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(live ? C_DIM : C_RED);
  canvas.setTextDatum(textdatum_t::middle_right);
  int w = canvas.drawString(status, 312, 12);
  canvas.fillSmoothCircle(312 - w - 8, 12, 3, live ? C_GREEN : C_RED);
}

void drawServingCard(const CellRx *serving, const Network *nets, int numNets) {
  const int x = 6, y = 26, w = 198, h = 92;
  canvas.fillSmoothRoundRect(x, y, w, h, 8, C_CARD);

  if (!serving) {
    canvas.setFont(&fonts::DejaVu12);
    canvas.setTextColor(C_DIM);
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.drawString(lastPacket ? "No serving cell" : "Waiting for beacon", x + w / 2, y + h / 2);
    return;
  }

  Quality q = rsrpQuality(serving->rsrp);

  // Operator name from the networks list, else MCC-MNC.
  char opName[20];
  snprintf(opName, sizeof(opName), "%u-%0*u", serving->mcc, serving->mnc3 ? 3 : 2, serving->mnc);
  for (int i = 0; i < numNets; i++) {
    if (nets[i].mcc == serving->mcc && nets[i].mnc == serving->mnc) {
      strncpy(opName, nets[i].name, sizeof(opName) - 1);
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
  canvas.setTextColor(C_TEXT);
  canvas.setTextDatum(textdatum_t::baseline_left);
  int nw = canvas.drawString(rsrp, x + 8, y + 64);
  canvas.setFont(&fonts::DejaVu12);
  canvas.setTextColor(C_DIM);
  canvas.drawString("dBm", x + 8 + nw + 4, y + 64);

  drawSignalBars(x + w - 62, y + 64, q.bars, q.color);

  char detail[48];
  snprintf(detail, sizeof(detail), "Band %u   PCI %u   RSRQ %d dB", serving->band, serving->pci, serving->rsrq);
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(C_DIM);
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

void drawNetworksCard(const Network *nets, int count) {
  const int x = 210, y = 26, w = 104, h = 138;
  canvas.fillSmoothRoundRect(x, y, w, h, 8, C_CARD);
  canvas.setFont(&fonts::DejaVu9);
  canvas.setTextColor(C_DIM);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString("NETWORKS", x + 9, y + 5);

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
    const Network &n = nets[i];

    char tech[12] = "";
    if (n.has4g) strcat(tech, "4G ");
    if (n.has3g) strcat(tech, "3G ");
    if (n.has2g) strcat(tech, "2G ");
    if (tech[0]) tech[strlen(tech) - 1] = '\0';
    canvas.setFont(&fonts::DejaVu9);
    canvas.setTextColor(n.has4g ? C_ACCENT : C_DIM);
    canvas.setTextDatum(textdatum_t::top_right);
    int tw = canvas.drawString(tech, x + w - 7, ry + 2);

    canvas.fillSmoothCircle(x + 11, ry + 6, 3, n.current ? C_GREEN : C_FAINT);
    canvas.setFont(&fonts::DejaVu12);
    char fitted[12];
    fitText(n.name, fitted, sizeof(fitted), w - 30 - tw);
    canvas.setTextColor(n.current ? C_GREEN : C_TEXT);
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.drawString(fitted, x + 19, ry);
  }
}

void drawScreen() {
  CellRx c[MAX_CELLS];
  OpRx o[MAX_OPERATORS];
  int rssi;
  unsigned long last, now = millis();

  portENTER_CRITICAL(&dataMux);
  for (auto &x : cells) if (x.used && now - x.seen > EXPIRE_MS) x.used = false;
  for (auto &x : ops) if (x.used && now - x.seen > EXPIRE_MS) x.used = false;
  memcpy(c, cells, sizeof(c));
  memcpy(o, ops, sizeof(o));
  rssi = beaconRssi;
  last = lastPacket;
  portEXIT_CRITICAL(&dataMux);

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

  // Merge operator records (one per technology) into one row per network,
  // the registered network first.
  Network nets[MAX_OPERATORS];
  int numNets = 0;
  for (auto &x : o) {
    if (!x.used) continue;
    int k = 0;
    while (k < numNets && !(nets[k].mcc == x.mcc && nets[k].mnc == x.mnc)) k++;
    if (k == numNets) {
      nets[k] = {x.mcc, x.mnc, "", false, false, false, false};
      strncpy(nets[k].name, x.name, sizeof(nets[k].name) - 1);
      numNets++;
    }
    if (x.stat == 2) nets[k].current = true;
    if (x.act == 7) nets[k].has4g = true;
    else if (x.act >= 2) nets[k].has3g = true;
    else nets[k].has2g = true;
  }
  std::sort(nets, nets + numNets, [](const Network &a, const Network &b) {
    if (a.current != b.current) return a.current;
    return a.has4g > b.has4g;
  });

  canvas.fillScreen(C_BG);
  drawHeader(now, last, rssi);
  drawServingCard(serving, nets, numNets);
  drawNeighbourCard(neigh, numNeigh);
  drawNetworksCard(nets, numNets);
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

  BLEDevice::init("");
  BLEScan *scan = BLEDevice::getScan();
  // wantDuplicates = true: the beacon reuses one address for every record.
  scan->setAdvertisedDeviceCallbacks(new BeaconCallbacks(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(100);  // window == interval: listen continuously
  scan->start(0, nullptr, false);  // 0 = scan forever, in the background
  Serial.println("Scanning for 7Semi LTE beacon...");
}

void loop() {
  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    drawScreen();
  }
}
