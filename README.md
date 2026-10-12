# 7_Semi

Embedded/IoT project built on the [7Semi ESP32-S3 EC200U 4G LTE Cat-1 WiFi
Bluetooth GNSS IoT Smart Modem](https://7semi.com/7semi-esp32-s3-ec200u-4g-lte-cat-1-wifi-bluetooth-gnss-iot-smart-modem/)
board.

## Hardware

- **MCU:** ESP32-S3-WROOM-1 (N8R8), dual-core LX7 @ 240 MHz, 8 MB flash, 8 MB PSRAM
- **Modem:** Quectel EC200U-CN — LTE-FDD B1/B3/B5/B8, LTE-TDD B34/B38/B39/B40/B41
- **GNSS:** GPS, BeiDou, Galileo, GLONASS, QZSS
- **Wireless:** 2.4 GHz WiFi, Bluetooth 5 LE
- **Power:** 5-26V supply input, GPIO at 3.3V

### ESP32 <-> EC200U UART wiring (onboard, fixed)

| EC200U Pin | ESP32 Pin    | Description            |
|------------|--------------|-------------------------|
| TX         | RX (GPIO 12) | Transmit from EC200U    |
| RX         | TX (GPIO 13) | Receive from ESP32      |
| GND        | GND          | Ground                  |

The user manual v2.1/2.2 lists GPIO 16/17, but on this board the modem only
answers on GPIO 12/13 at 115200 baud (found with `AT_Command_Tester/Pin_Probe`).
Other modem connections found by probing:

| ESP32 Pin | Function                                                          |
|-----------|-------------------------------------------------------------------|
| GPIO 9    | Modem hardware reset: a 300 ms HIGH pulse reboots the EC200U (it prints `RDY` ~4 s later). Idles LOW. Found with `AT_Command_Tester/Reset_Probe`. |
| GPIO 1, 2 | Pulled high on the board; purpose unknown. Pulsing GPIO 1 does nothing, and neither pin changes when the modem reboots. |

### Mobile data

The modem ships with no APN configured. On an O2 UK SIM a blank APN works:
`AT+QICSGP=1,1,"","","",0` then `AT+QIACT=1` gives the context an IP.
Verified with `AT+QNTP` (time from pool.ntp.org), `AT+QPING` to 8.8.8.8
(~57 ms) and `AT+QHTTPGET` of http://httpbin.org/get (HTTP 200).

`LTE_Scanner_Beacon` checks this by itself every 2 minutes: it activates the
data context if needed (set `DATA_APN` for a non-O2 SIM) and fetches the time
from pool.ntp.org. The display shows the result in its header ("web 368 ms")
and the modem's data counter (`AT+QGDCNT?`) in its bottom-left card
("TX 3.2k  RX 3.9k").

### Power

Connect **both** USB-C ports. One USB port alone powers only the ESP32; the
EC200U stays silent (no reply to AT, no Quectel USB ports on the PC) until
its own port is connected too.

### Arduino IDE setup notes (from manufacturer manual)

- Tools -> USB CDC on Boot -> **Enabled**
- If using the Serial Monitor, reset the ESP32-S3 after uploading code.

## Sketches

- `AT_Command_Tester/` - UART passthrough between Serial Monitor and the
  EC200U modem, for sending AT commands and reading responses directly.
  First bring-up step for this board.
- `GPS_Tracker/` - Enables the EC200U GNSS engine (`AT+QGPS=1`) and polls
  `AT+QGPSLOC=2` every 5 s, printing lat/lon, altitude, speed and satellite
  count. Requires the GNSS antenna and open sky.
- `LTE_Scanner_Beacon/` - Runs on this board. Reads the serving and
  neighbour LTE cells (`AT+QENG`) every 5 s and every operator in range with
  its strongest cells (`AT+QOPS`, ~40 s) every 3 min, checks the internet
  connection every 2 min, and once a second broadcasts everything as one
  BLE 5 extended advertisement at +20 dBm. Recovers the modem by itself:
  re-initialises it after a reboot or repeated timeouts, and hard-resets it
  through GPIO 9 if it stops answering entirely.
- `LTE_Scanner_Display/` - Runs on a LilyGO T-Display-S3. Receives the
  beacon's snapshots and draws a dashboard: serving cell (RSRP in dBm, signal
  bars, quality rating), neighbour cells, visible networks with their best
  signal, and modem / internet status. Needs the LovyanGFX library. Board
  settings: ESP32S3 Dev Module, USB CDC On Boot Enabled, PSRAM OPI, Flash 16MB.

Both LTE scanner sketches target Arduino-ESP32 core 3.x and need the
**NimBLE-Arduino** library (2.x) with extended advertising enabled: in its
`src/nimconfig.h`, uncomment `#define CONFIG_BT_NIMBLE_EXT_ADV 1`. (The
core's built-in BLE library is compiled without extended advertising.) Both
boards are ESP32-S3, which supports BLE 5; the original ESP32 does not.

## BLE advert format

Protocol version 4. The beacon sends one non-connectable BLE 5 extended
advertisement (1M PHY, every 100 ms, +20 dBm) whose manufacturer-specific
data (AD type 0xFF) is a complete snapshot, rebuilt every second. The
receiver replaces everything it shows with each snapshot, so a cell the
modem stops reporting disappears with the next one. A snapshot is at most
247 bytes, so it fits one advertising PDU without chaining. All multi-byte
fields are little-endian.

(Version 1 sent one 31-byte legacy advert per record in rotation; the
status record alone had filled the 26 usable bytes. Version 2 introduced
the snapshot; version 3 added the data byte counters and NTP time; version 4 added GNSS.)

Header and status, 50 bytes:

| Bytes | Field                                                                 |
|-------|-----------------------------------------------------------------------|
| 0-1   | Company ID `0xFFFF` (Bluetooth SIG test ID)                           |
| 2-3   | Magic `'7' 'S'`                                                       |
| 4     | Protocol version (4)                                                  |
| 5     | Snapshot sequence number                                              |
| 6     | Modem state: 0 starting, 1 SIM not ready, 2 searching, 3 registered, 4 not responding |
| 7     | Flags: bit 0 network scan (`AT+QOPS`) in progress                     |
| 8     | CSQ signal (0-31, 99 = unknown)                                       |
| 9-10  | Seconds since the last good serving-cell reading (0xFFFF = never)     |
| 11-12 | Seconds since the last good network scan (0xFFFF = never)             |
| 13-16 | Beacon uptime, seconds                                                |
| 17    | Number of times the beacon has re-initialised the modem (soft)       |
| 18    | Number of times the beacon has hard-reset the modem (GPIO 9)         |
| 19    | Internet check: 0 not checked, 1 OK, 2 no IP (APN/activation failed), 3 NTP failed |
| 20-21 | Seconds since the last good internet check (0xFFFF = never)          |
| 22-23 | NTP round trip of the last good check, ms                            |
| 24-27 | Mobile data bytes sent (modem counter, `AT+QGDCNT?`)                  |
| 28-31 | Mobile data bytes received                                            |
| 32-35 | UTC time from the last good NTP check, seconds since 1970 (0 = none)  |
| 36    | GNSS state: 0 off, 1 searching (no fix), 2 fix 2D, 3 fix 3D          |
| 37    | Satellites used in the fix (`AT+QGPSLOC`)                             |
| 38    | Satellites in view, summed over all constellations (NMEA GSV)         |
| 39    | HDOP * 10                                                             |
| 40-41 | Seconds since the last GNSS poll (0xFFFF = never)                     |
| 42-45 | Latitude, int32, degrees * 1e6 (0 without a fix)                      |
| 46-49 | Longitude, int32, degrees * 1e6 (0 without a fix)                     |

Cells: a count byte (0-8), then, if non-zero, a 9-byte block for the serving
operator followed by 9 bytes per cell (serving cell first, then neighbours
strongest first). Neighbours are configured by the serving operator, so all
cells share its MCC/MNC.

| Bytes | Field                                                       |
|-------|-------------------------------------------------------------|
| 0-1   | MCC                                                         |
| 2-3   | MNC                                                         |
| 4     | Flags: bit 2 three-digit MNC                                |
| 5-8   | Serving cell ID (0 if no serving cell)                      |

Per cell:

| Bytes | Field                                                       |
|-------|-------------------------------------------------------------|
| 0     | Bits 0-1 kind (0 serving, 1 intra-freq neighbour, 2 inter-freq neighbour), bit 3 TDD |
| 1     | LTE band                                                    |
| 2-3   | EARFCN                                                      |
| 4-5   | Physical cell ID (PCI)                                      |
| 6     | RSRP, int8 dBm                                              |
| 7     | RSRQ, int8 dB                                               |
| 8     | SINR, int8 dB (serving cell only; -128 = unknown)           |

SINR comes from the modem as X with dB = X / 2 - 23.5 (EC200U AT manual).
Neighbour lines carry a similar-looking field that stays fixed per cell, so
neighbours report SINR as unknown.

Networks: a count byte (0-6), then per operator from `AT+QOPS`:

| Bytes | Field                                                       |
|-------|-------------------------------------------------------------|
| 0-1   | MCC                                                         |
| 2-3   | MNC                                                         |
| 4     | Flags: bit 2 three-digit MNC, bit 4 has 4G, bit 5 has 3G, bit 6 has 2G |
| 5     | Strongest 4G cell RSRP, int8 dBm (0 = no 4G cell)           |
| 6     | LTE band of that cell                                       |
| 7     | Number of distinct 4G cells seen                            |
| 8     | Strongest 2G cell RSSI, int8 dBm (0 = no 2G cell)           |
| 9     | Name length n (0-9)                                         |
| 10-   | Operator short name (else MCC-MNC), n ASCII chars           |

The registered network is the one whose MCC/MNC matches the serving cell.

## Reference docs

- [ESP32-S3 EC200U board user manual](https://7semi.com/content/datasheets/ESP32-S3-EC200U-2.2.pdf)
- [EC200U-CN datasheet](https://7semi.com/content/datasheets/EC200U-CN.pdf)
- [EC200U AT commands manual](https://7semi.com/content/datasheets/EC200U-AT.pdf)
- [EC200U GNSS AT commands](https://7semi.com/content/datasheets/EC200U-GNSS.pdf)
- [EC200U MQTT application note](https://7semi.com/content/datasheets/EC200U-MQTT.pdf)
- [EC200U TCP/IP application note](https://7semi.com/content/datasheets/EC200U%20TCP-IP.pdf)
- [ESP32-S3-WROOM-1 datasheet](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)
