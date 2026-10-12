// Wire format of the beacon snapshot: constants and little-endian byte
// helpers. The layout is documented in README.md.
#pragma once
#include <stdint.h>

#define COMPANY_ID 0xFFFF  // Bluetooth SIG ID reserved for testing
#define MAGIC0 '7'
#define MAGIC1 'S'
// v1 was one 31-byte legacy advert per record; v2 the snapshot; v3 adds the
// mobile data byte counters and the last NTP time to the status block; v4
// adds the GNSS fix state, satellite counts and position.
// v5 adds SIM restarts, data re-activations and GNSS search / first-fix times.
#define PROTOCOL_VERSION 5
#define SNAP_MAX_CELLS 8
#define SNAP_MAX_NETS 6
#define SNAP_MAX_BYTES 256  // 56 header+status, 1+9+8*9 cells, 1+6*19 networks = 253

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

#define GNSS_OFF 0        // engine not started / modem not answering
#define GNSS_SEARCHING 1  // running, no fix yet
#define GNSS_FIX_2D 2
#define GNSS_FIX_3D 3

#define NET_FLAG_MNC3 0x04
#define NET_FLAG_4G 0x10
#define NET_FLAG_3G 0x20
#define NET_FLAG_2G 0x40

#define KIND_SERVING 0
#define KIND_INTRA 1  // neighbour on the same frequency
#define KIND_INTER 2  // neighbour on another frequency


// Seconds since 1970-01-01 UTC for a civil date and time (days-from-civil
// algorithm, valid for any Gregorian date).
inline uint32_t utcEpoch(int y, int mon, int d, int h, int minute, int s) {
  y -= mon <= 2;
  int era = y / 400;
  int yoe = y - era * 400;
  int doy = (153 * (mon + (mon > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468;
  return (uint32_t)(days * 86400L + h * 3600L + minute * 60L + s);
}

inline void put16(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = v >> 8;
}

inline void put32(uint8_t *p, uint32_t v) {
  for (int b = 0; b < 4; b++) p[b] = (v >> (8 * b)) & 0xFF;
}

inline uint16_t ageSeconds(unsigned long t, unsigned long now) {
  if (t == 0) return 0xFFFF;  // never
  return (uint16_t)min((now - t) / 1000, 0xFFFEUL);
}
