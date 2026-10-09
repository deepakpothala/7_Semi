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
| TX         | RX (GPIO 16) | Transmit from EC200U    |
| RX         | TX (GPIO 17) | Receive from ESP32      |
| GND        | GND          | Ground                  |

Source: 7Semi ESP32-S3 EC200U 4G User Manual v2.1/2.2.

### Arduino IDE setup notes (from manufacturer manual)

- Tools -> USB CDC on Boot -> **Enabled**
- If using the Serial Monitor, reset the ESP32-S3 after uploading code.

## Sketches

- `AT_Command_Tester/` - UART passthrough between Serial Monitor and the
  EC200U modem, for sending AT commands and reading responses directly.
  First bring-up step for this board.

## Reference docs

- [ESP32-S3 EC200U board user manual](https://7semi.com/content/datasheets/ESP32-S3-EC200U-2.2.pdf)
- [EC200U-CN datasheet](https://7semi.com/content/datasheets/EC200U-CN.pdf)
- [EC200U AT commands manual](https://7semi.com/content/datasheets/EC200U-AT.pdf)
- [EC200U GNSS AT commands](https://7semi.com/content/datasheets/EC200U-GNSS.pdf)
- [EC200U MQTT application note](https://7semi.com/content/datasheets/EC200U-MQTT.pdf)
- [EC200U TCP/IP application note](https://7semi.com/content/datasheets/EC200U%20TCP-IP.pdf)
- [ESP32-S3-WROOM-1 datasheet](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)
