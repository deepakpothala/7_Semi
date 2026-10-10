// 7Semi ESP32-S3 EC200U: GNSS position reader.
// Turns on the EC200U's GNSS engine and polls AT+QGPSLOC every few seconds,
// printing latitude/longitude, altitude, speed and satellite count once a
// fix is available. Needs the GNSS antenna connected and a clear sky view;
// a cold-start fix can take 30 s to a few minutes.

#define MODEM_RX_PIN 12  // ESP32 RX <- EC200U TX (manual says 16; measured 12)
#define MODEM_TX_PIN 13  // ESP32 TX -> EC200U RX (manual says 17; measured 13)
#define MODEM_BAUD 115200
#define POLL_INTERVAL_MS 5000

// Sends an AT command and collects the reply until "OK", "ERROR" or timeout.
String sendAT(const String &cmd, unsigned long timeoutMs = 2000) {
  while (Serial1.available()) Serial1.read();
  Serial1.print(cmd);
  Serial1.print("\r\n");

  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial1.available()) {
      resp += (char)Serial1.read();
    }
    if (resp.indexOf("OK\r\n") >= 0 || resp.indexOf("ERROR") >= 0) break;
  }
  resp.trim();
  return resp;
}

bool waitForModem() {
  for (int i = 0; i < 10; i++) {
    if (sendAT("AT", 1000).indexOf("OK") >= 0) return true;
    delay(1000);
  }
  return false;
}

// Splits a comma-separated field list into out[], returns the field count.
int splitFields(const String &s, String out[], int maxFields) {
  int count = 0, from = 0;
  while (count < maxFields) {
    int comma = s.indexOf(',', from);
    if (comma < 0) {
      out[count++] = s.substring(from);
      break;
    }
    out[count++] = s.substring(from, comma);
    from = comma + 1;
  }
  return count;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  delay(2000);
  Serial.println("7Semi ESP32-S3 EC200U - GPS Tracker");

  if (!waitForModem()) {
    Serial.println("Modem not responding to AT. Check power and UART pins.");
    return;
  }
  Serial.println("Modem OK");

  // Enable all constellations the EC200U supports (GPS+BeiDou+Galileo+GLONASS).
  Serial.println(sendAT("AT+QGPSCFG=\"gnssconfig\",1"));

  // +CME ERROR: 504 means the GNSS session is already running, which is fine.
  String r = sendAT("AT+QGPS=1");
  Serial.println(r.indexOf("504") >= 0 ? "GNSS already running" : r);
  Serial.println("Waiting for fix...");
}

void loop() {
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll < POLL_INTERVAL_MS) return;
  lastPoll = millis();

  // Mode 2 returns lat/lon in signed decimal degrees.
  String resp = sendAT("AT+QGPSLOC=2", 3000);

  int idx = resp.indexOf("+QGPSLOC: ");
  if (idx < 0) {
    if (resp.indexOf("516") >= 0) {
      Serial.println("No fix yet (516) - keep antenna under open sky");
    } else {
      Serial.print("QGPSLOC error: ");
      Serial.println(resp);
    }
    return;
  }

  // +QGPSLOC: <UTC>,<lat>,<lon>,<HDOP>,<alt>,<fix>,<COG>,<spkm>,<spkn>,<date>,<nsat>
  String line = resp.substring(idx + 10);
  int eol = line.indexOf('\r');
  if (eol >= 0) line = line.substring(0, eol);

  String f[11];
  if (splitFields(line, f, 11) < 11) {
    Serial.print("Unexpected QGPSLOC format: ");
    Serial.println(line);
    return;
  }

  Serial.printf("UTC %s %s  Lat %s  Lon %s  Alt %s m  HDOP %s  Fix %sD  Speed %s km/h  Sats %s\n",
                f[9].c_str(), f[0].c_str(), f[1].c_str(), f[2].c_str(), f[4].c_str(),
                f[3].c_str(), f[5].c_str(), f[7].c_str(), f[10].c_str());
  Serial.printf("Map: https://maps.google.com/?q=%s,%s\n", f[1].c_str(), f[2].c_str());
}
