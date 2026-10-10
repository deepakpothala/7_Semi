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
GPIO 1 also connects to the modem; driving it as an output restarted the
modem during probing, so leave it alone.

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
  neighbour LTE cells (`AT+QENG`) every 5 s and the visible operators
  (`AT+COPS=?`) every 10 min, then broadcasts each record as a BLE
  advertisement, rotating every 500 ms.
- `LTE_Scanner_Display/` - Runs on a LilyGO T-Display-S3. Listens for the
  beacon's adverts and draws a dashboard: serving cell (RSRP in dBm, signal
  bars, quality rating), neighbour cells, and visible networks. Needs the
  LovyanGFX library. Board settings: ESP32S3
  Dev Module, USB CDC On Boot Enabled, PSRAM OPI, Flash 16MB.

Both LTE scanner sketches target Arduino-ESP32 core 3.x.

## BLE advert format

Manufacturer-specific data (AD type 0xFF), all multi-byte fields little-endian:

| Bytes | Field                                                        |
|-------|--------------------------------------------------------------|
| 0-1   | Company ID `0xFFFF` (Bluetooth SIG test ID)                  |
| 2-3   | Magic `'7' 'S'`                                              |
| 4     | Type: 1 = LTE cell, 2 = operator, 3 = beacon status          |
| 5     | Scan sequence number (increments each scan of that type)     |
| 6-7   | Record index, record count                                   |

Cells and operators continue with:

| Bytes | Field                                                        |
|-------|--------------------------------------------------------------|
| 8-9   | MCC                                                          |
| 10-11 | MNC                                                          |
| 12    | Flags: bits 0-1 cell kind (0 serving, 1 intra-freq neighbour, 2 inter-freq neighbour), bit 2 three-digit MNC, bit 3 TDD |

Cell (type 1), 25 bytes total:

| Bytes | Field                      |
|-------|----------------------------|
| 13    | LTE band                   |
| 14-15 | EARFCN                     |
| 16-17 | Physical cell ID (PCI)     |
| 18    | RSRP, int8 dBm             |
| 19    | RSRQ, int8 dB              |
| 20    | SINR, int8 dB              |
| 21-24 | Cell ID (serving cell only, else 0) |

Operator (type 2):

| Bytes | Field                                                       |
|-------|-------------------------------------------------------------|
| 13    | Status (3GPP 27.007): 1 available, 2 current, 3 forbidden   |
| 14    | Access tech: 0 GSM, 2 UTRAN, 7 LTE                          |
| 15-   | Operator short name, up to 11 ASCII chars, no terminator    |

Beacon status (type 3), 20 bytes total, sent once per rotation (even when
there is no cell or operator data, so the receiver knows the beacon is alive):

| Bytes | Field                                                                 |
|-------|-----------------------------------------------------------------------|
| 8     | Modem state: 0 starting, 1 SIM not ready, 2 searching, 3 registered, 4 not responding |
| 9     | Flags: bit 0 operator scan (`AT+COPS=?`) in progress                  |
| 10    | CSQ signal (0-31, 99 = unknown)                                       |
| 11-12 | Seconds since the last good serving-cell reading (0xFFFF = never)     |
| 13-14 | Seconds since the last good operator scan (0xFFFF = never)            |
| 15-18 | Beacon uptime, seconds                                                |
| 19    | Number of times the beacon has re-initialised the modem              |

## Reference docs

- [ESP32-S3 EC200U board user manual](https://7semi.com/content/datasheets/ESP32-S3-EC200U-2.2.pdf)
- [EC200U-CN datasheet](https://7semi.com/content/datasheets/EC200U-CN.pdf)
- [EC200U AT commands manual](https://7semi.com/content/datasheets/EC200U-AT.pdf)
- [EC200U GNSS AT commands](https://7semi.com/content/datasheets/EC200U-GNSS.pdf)
- [EC200U MQTT application note](https://7semi.com/content/datasheets/EC200U-MQTT.pdf)
- [EC200U TCP/IP application note](https://7semi.com/content/datasheets/EC200U%20TCP-IP.pdf)
- [ESP32-S3-WROOM-1 datasheet](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)
