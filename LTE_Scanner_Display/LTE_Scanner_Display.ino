// LilyGO T-Display-S3: receiver for LTE_Scanner_Beacon.
// Passively scans BLE for the 7Semi beacon's adverts, collects the operator
// and LTE cell records, and shows them on the 320x170 display, cells sorted
// by signal strength.
//
// Built for Arduino-ESP32 core 3.x. Needs the "GFX Library for Arduino"
// (Arduino_GFX by moononournation) library.
// Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled, PSRAM: OPI PSRAM.
// Advert format: see README.md "BLE advert format".

#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <algorithm>

#define PIN_LCD_POWER 15
#define PIN_LCD_BL 38

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
#define REDRAW_MS 500

// T-Display-S3: ST7789 170x320 on an 8-bit parallel bus.
Arduino_DataBus *bus = new Arduino_ESP32PAR8Q(
  7 /* DC */, 6 /* CS */, 8 /* WR */, 9 /* RD */,
  39, 40, 41, 42, 45, 46, 47, 48 /* D0-D7 */);
Arduino_GFX *panel = new Arduino_ST7789(bus, 5 /* RST */, 1 /* landscape */, true /* IPS */,
                                        170, 320, 35, 0, 35, 0);
Arduino_Canvas *gfx = new Arduino_Canvas(320, 170, panel);  // draw off-screen, no flicker

const uint16_t C_BLACK = 0x0000, C_WHITE = 0xFFFF, C_GREY = 0x7BEF, C_CYAN = 0x07FF,
               C_GREEN = 0x07E0, C_YELLOW = 0xFFE0, C_ORANGE = 0xFD20, C_RED = 0xF800;

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

static CellRx cells[MAX_CELLS];
static OpRx ops[MAX_OPERATORS];
static int beaconRssi = 0;
static unsigned long lastPacket = 0;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

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

uint16_t rsrpColor(int rsrp) {
  if (rsrp >= -90) return C_GREEN;
  if (rsrp >= -105) return C_YELLOW;
  if (rsrp >= -115) return C_ORANGE;
  return C_RED;
}

const char *techName(uint8_t act) {
  if (act == 7) return "4G";
  if (act >= 2) return "3G";
  return "2G";
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

  // Strongest cell first; unused slots sink to the end.
  std::sort(c, c + MAX_CELLS, [](const CellRx &a, const CellRx &b) {
    if (a.used != b.used) return a.used;
    return a.rsrp > b.rsrp;
  });

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(4, 2);
  gfx->print("LTE Scanner");

  char status[32];
  if (last == 0) snprintf(status, sizeof(status), "waiting for beacon");
  else snprintf(status, sizeof(status), "BLE %d dBm  %lus ago", rssi, (now - last) / 1000);
  gfx->setTextSize(1);
  gfx->setTextColor(last && now - last < 5000 ? C_GREY : C_RED);
  gfx->setCursor(316 - strlen(status) * 6, 6);
  gfx->print(status);
  gfx->drawFastHLine(0, 20, 320, C_GREY);

  // Operators: two columns, "*" marks the one the SIM is registered on.
  int y = 24, col = 0;
  for (auto &op : o) {
    if (!op.used) continue;
    gfx->setTextColor(op.stat == 2 ? C_GREEN : op.stat == 3 ? C_RED : C_WHITE);
    gfx->setCursor(col ? 164 : 4, y);
    gfx->printf("%c%-11s %u-%0*u %s", op.stat == 2 ? '*' : ' ', op.name, op.mcc,
                op.mnc3 ? 3 : 2, op.mnc, techName(op.act));
    if (col) y += 10;
    col ^= 1;
  }
  if (col) y += 10;
  if (y > 24) {
    gfx->drawFastHLine(0, y + 1, 320, C_GREY);
    y += 5;
  }

  // Cells: S = serving, N = neighbour (same freq), X = neighbour (other freq).
  gfx->setTextSize(2);
  if (!c[0].used) {
    gfx->setTextColor(C_GREY);
    gfx->setCursor(4, y + 4);
    gfx->print("No LTE cells yet");
  }
  for (int i = 0; i < MAX_CELLS && c[i].used && y + 16 <= 170; i++) {
    const CellRx &cell = c[i];
    char kind = cell.kind == KIND_SERVING ? 'S' : cell.kind == KIND_INTRA ? 'N' : 'X';
    gfx->setTextColor(cell.kind == KIND_SERVING ? C_CYAN : C_WHITE);
    gfx->setCursor(4, y);
    gfx->printf("%c B%-2u %3u %4d", kind, cell.band, cell.pci, cell.rsrp);

    // Bar spans RSRP -140 (empty) .. -44 dBm (full).
    const int bx = 168, bw = 148;
    int fill = constrain((cell.rsrp + 140) * bw / 96, 0, bw);
    gfx->drawRect(bx, y, bw, 14, C_GREY);
    gfx->fillRect(bx + 1, y + 1, max(fill - 2, 0), 12, rsrpColor(cell.rsrp));
    y += 18;
  }

  gfx->flush();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LCD_POWER, OUTPUT);
  digitalWrite(PIN_LCD_POWER, HIGH);
  if (!gfx->begin()) Serial.println("Display init failed");
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);
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
