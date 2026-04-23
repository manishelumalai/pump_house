/*************************************************************************
  PROJECT: Bharat Pi 4G Board - Robust MQTT Float + Pump Control (Production)
  AUTHOR: Modified for Manish by GPT-5 (stable version B) - corrected
  NOTES:
    - Avoids restarting the ESP unless modem power-cycle fails.
    - Uses modem.maintain() and periodic TLS/MQTT reconnects.
    - Non-blocking loop using millis().
*************************************************************************/

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#define SerialAT  Serial1
#define SerialMon Serial
#define UART_BAUD 115200

#define PIN_TX 17
#define PIN_RX 16
#define PWR_PIN 26
#define LED_PIN 2

#include <TinyGsmClient.h>
#include "SSLClient.h"
#include <PubSubClient.h>

// ---------- APN (edit for your SIM) ----------
const char apn[] = "airtelgprs.com"; // change to your SIM's APN

// ---------- HiveMQ Cloud MQTT ----------
const char* mqtt_server   = "8e3ddd6ba80a4e3e99739281bebb36d8.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;
const char* mqtt_username = "Sandhep";
const char* mqtt_password = "Sandhep13";

const char* clientID    = "BharatPi-ESP32";
const char* topic_pump  = "Pumphouse/Domestic-WaterTank/Pump_State";
const char* topic_oht   = "Pumphouse/Domestic-WaterTank/OHT";
const char* topic_ugt   = "Pumphouse/Domestic-WaterTank/UGT";
const char* lwt_topic   = "Pumphouse/LWT/Status";
const char* lwt_message = "Offline";
const int   qos         = 1;
const bool  lwt_retain  = true;

const unsigned long KEEPALIVE_TIME = 30; // seconds <-- renamed to avoid macro conflict

// ---------- Pins ----------
#define OHT_FLOAT_PIN   19
#define UGT_FLOAT_PIN   18
#define PUMP_RELAY_PIN  27

// ---------- Root CA ----------
const char root_ca[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFBjCCAu6gAwIBAgIRAMISMktwqbSRcdxA9+KFJjwwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw
WhcNMjcwMzEyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg
RW5jcnlwdDEMMAoGA1UEAxMDUjEyMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB
CgKCAQEA2pgodK2+lP474B7i5Ut1qywSf+2nAzJ+Npfs6DGPpRONC5kuHs0BUT1M
5ShuCVUxqqUiXXL0LQfCTUA83wEjuXg39RplMjTmhnGdBO+ECFu9AhqZ66YBAJpz
kG2Pogeg0JfT2kVhgTU9FPnEwF9q3AuWGrCf4yrqvSrWmMebcas7dA8827JgvlpL
Thjp2ypzXIlhZZ7+7Tymy05v5J75AEaz/xlNKmOzjmbGGIVwx1Blbzt05UiDDwhY
XS0jnV6j/ujbAKHS9OMZTfLuevYnnuXNnC2i8n+cF63vEzc50bTILEHWhsDp7CH4
WRt/uTp8n1wBnWIEwii9Cq08yhDsGwIDAQABo4H4MIH1MA4GA1UdDwEB/wQEAwIB
hjAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwEgYDVR0TAQH/BAgwBgEB
/wIBADAdBgNVHQ4EFgQUALUp8i2ObzHom0yteD763OkM0dIwHwYDVR0jBBgwFoAU
ebRZ5nu25eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAC
hhZodHRwOi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcG
A1UdHwQgMB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcN
AQELBQADggIBAI910AnPanZIZTKS3rVEyIV29BWEjAK/duuz8eL5boSoVpHhkkv3
4eoAeEiPdZLj5EZ7G2ArIK+gzhTlRQ1q4FKGpPPaFBSpqV/xbUb5UlAXQOnkHn3m
FVj+qYv87/WeY+Bm4sN3Ox8BhyaU7UAQ3LeZ7N1X01xxQe4wIAAE3JVLUCiHmZL+
qoCUtgYIFPgcg350QMUIWgxPXNGEncT921ne7nluI02V8pLUmClqXOsCwULw+PVO
ZCB7qOMxxMBoCUeL2Ll4oMpOSr5pJCpLN3tRA2s6P1KLs9TSrVhOk+7LX28NMUlI
usQ/nxLJID0RhAeFtPjyOCOscQBA53+NRjSCak7P4A5jX7ppmkcJECL+S0i3kXVU
y5Me5BbrU8973jZNv/ax6+ZK6TM8jWmimL6of6OrX7ZU6E2WqazzsFrLG3o2kySb
zlhSgJ81Cl4tv3SbYiYXnJExKQvzf83DYotox3f0fwv7xln1A2ZLplCb0O+l/AK0
YE0DS2FPxSAHi0iwMfW2nNHJrXcY3LLHD77gRgje4Eveubi2xxa+Nmk/hmhLdIET
iVDFanoCrMVIpQ59XWHkzdFmoHXHBV7oibVjGSO7ULSQ7MJ1Nz51phuDJSgAIU7A
0zrLnOrAj/dfrlEWRhCvAgbuwLZX1A2sjNjXoPOHbsPiy+lO1KF8/XY7
-----END CERTIFICATE-----
)EOF";

// ---------- Clients ----------
TinyGsm       modem(SerialAT);
TinyGsmClient gsm(modem);
SSLClient     secure_client(&gsm);   // keep as before
PubSubClient  client(secure_client);

// ---------- Debounce variables ----------
bool lastOHT = HIGH, lastUGT = HIGH;
bool stableOHT = HIGH, stableUGT = HIGH;
unsigned long lastDebounceTimeOHT = 0, lastDebounceTimeUGT = 0;
const unsigned long debounceDelay = 1000; // ms

// ---------- Timing & recovery configuration ----------
unsigned long lastModemMaintain = 0;
const unsigned long MODEM_MAINTAIN_INTERVAL = 15UL * 1000UL; // 15s

unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 15UL * 1000UL; // 15s initial backoff

unsigned long lastTlsReconnect = 0;
const unsigned long TLS_RECONNECT_INTERVAL = 60UL * 60UL * 1000UL; // 1 hour: periodic TLS reset

unsigned long lastModemHardReset = 0;
const unsigned long MODEM_HARD_RESET_INTERVAL = 6UL * 60UL * 60UL * 1000UL; // 6 hours (tunable)

// exponential backoff cap (ms)
const unsigned long MAX_RECONNECT_BACKOFF = 2UL * 60UL * 1000UL; // 2 minutes

// ---------- State ----------
unsigned long mqttBackoff = 2000; // start 2s
bool initialSetupDone = false;

// ---------- Helper Functions ----------
void modemPowerOn() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
  delay(2000);
}

void modemPowerCycle() {
  Serial.println("[MOD] Power-cycle modem (PWR pin)...");
  digitalWrite(PWR_PIN, LOW);  // power off
  delay(2500);
  digitalWrite(PWR_PIN, HIGH); // power on
  delay(8000);                 // allow boot (tune if needed)
  Serial.println("[MOD] Power-cycle complete. Re-init modem.");
}

void modemFullInit() {
  Serial.println("[MOD] Initializing modem library...");
  // init sequence
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);
  modem.init();
}

void ensureGprsConnected() {
  unsigned long start = millis();
  modem.gprsConnect(apn, "", "");
  while (!modem.isGprsConnected()) {
    if (millis() - start > 30000UL) { // 30s timeout -> try modem power cycle
      Serial.println("\n[ERROR] GPRS did not connect within 30s -> modem power-cycle");
      modemPowerCycle();
      modemFullInit();
      start = millis();
      modem.gprsConnect(apn, "", "");
    }
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n[OK] GPRS connected.");
}

// MQTT connect with LWT and TLS keepalive
bool mqttConnect() {
  Serial.print("[MQTT] Attempting connection...");
  // setup LWT
  bool ok = client.connect(clientID,
                           mqtt_username,
                           mqtt_password,
                           lwt_topic,
                           qos,
                           lwt_retain,
                           lwt_message);
  if (ok) {
    Serial.println("connected");
    // publish online LWT and subscribe pump topic
    client.publish(lwt_topic, "Online", true);
    client.subscribe(topic_pump);
    client.setKeepAlive((uint16_t)KEEPALIVE_TIME); // set keepalive safely
    mqttBackoff = 2000; // reset backoff
    return true;
  } else {
    Serial.printf("failed, rc=%d\n", client.state());
    return false;
  }
}

void safeMqttDisconnect() {
  if (client.connected()) {
    Serial.println("[MQTT] Disconnecting safely...");
    client.disconnect();
    delay(200);
  }
}

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  Serial.printf("[MQTT] %s => %s\n", topic, message.c_str());

  if (String(topic) == topic_pump) {
    if (message == "ON") {
      digitalWrite(PUMP_RELAY_PIN, LOW);
      Serial.println("[PUMP] ON");
    } else {
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      Serial.println("[PUMP] OFF");
    }
  }
}

// ---------- Setup ----------
void setup() {
  SerialMon.begin(115200);
  delay(10);

  pinMode(LED_PIN, OUTPUT);
  pinMode(OHT_FLOAT_PIN, INPUT_PULLUP);
  pinMode(UGT_FLOAT_PIN, INPUT_PULLUP);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, HIGH); // pump off (active LOW assumed)

  // modem power on & init
  modemPowerOn();
  modemFullInit();

  // set CA for SSL client
  secure_client.setCACert(root_ca);

  Serial.println("[SYS] Connecting to mobile network (GPRS)...");
  ensureGprsConnected();

  // configure MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive((uint16_t)KEEPALIVE_TIME); // set keepalive here too

  // small LED blink to show ready
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  initialSetupDone       = true;
  lastModemMaintain      = millis();
  lastMqttReconnectAttempt = millis();
  lastTlsReconnect       = millis();
  lastModemHardReset     = millis();
}

// ---------- Float change + MQTT ----------
void checkAndPublishFloatStatus() {
  int readingOHT = digitalRead(OHT_FLOAT_PIN);
  int readingUGT = digitalRead(UGT_FLOAT_PIN);
  unsigned long currentTime = millis();

  // --- OHT ---
  if (readingOHT != lastOHT) lastDebounceTimeOHT = currentTime;

  if ((currentTime - lastDebounceTimeOHT) > debounceDelay && readingOHT != stableOHT) {
    stableOHT = readingOHT;
    String msg = (stableOHT == LOW) ? "FULL" : "EMPTY";
    if (client.connected()) client.publish(topic_oht, msg.c_str(), true);
    Serial.printf("[SENSOR] OHT → %s\n", msg.c_str());
  }
  lastOHT = readingOHT;

  // --- UGT ---
  if (readingUGT != lastUGT) lastDebounceTimeUGT = currentTime;

  if ((currentTime - lastDebounceTimeUGT) > debounceDelay && readingUGT != stableUGT) {
    stableUGT = readingUGT;
    String msg = (stableUGT == LOW) ? "FULL" : "EMPTY";
    if (client.connected()) client.publish(topic_ugt, msg.c_str(), true);
    Serial.printf("[SENSOR] UGT → %s\n", msg.c_str());
  }
  lastUGT = readingUGT;
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  // 1) TinyGSM maintenance frequently prevents PDP drop
  if (now - lastModemMaintain >= MODEM_MAINTAIN_INTERVAL) {
    lastModemMaintain = now;
    modem.maintain();
  }

  // 2) Ensure MQTT connected, use backoff; but do not block
  if (!client.connected()) {
    if (now - lastMqttReconnectAttempt >= mqttBackoff) {
      lastMqttReconnectAttempt = now;
      Serial.println("[MQTT] Not connected. Trying to reconnect...");

      // Ensure GPRS is present before attempting MQTT
      if (!modem.isGprsConnected()) {
        Serial.println("[MQTT] GPRS not connected. Attempting GPRS connect...");
        ensureGprsConnected();
      }

      // Attempt MQTT connect
      if (mqttConnect()) {
        // success
        mqttBackoff = 2000;
      } else {
        // failed -> increase backoff
        mqttBackoff = mqttBackoff * 2;
        if (mqttBackoff > MAX_RECONNECT_BACKOFF) mqttBackoff = MAX_RECONNECT_BACKOFF;
        Serial.printf("[MQTT] Reconnect failed, next attempt in %lu ms\n", mqttBackoff);
      }
    }
  } else {
    // If connected, run client loop to keep connection alive and handle callbacks
    client.loop();
  }

  // 3) Periodic TLS reset to avoid SSL leaks on SIM7600 (do safe reconnect)
  if (now - lastTlsReconnect >= TLS_RECONNECT_INTERVAL) {
    lastTlsReconnect = now;
    Serial.println("[TLS] Periodic TLS reset: disconnecting and reconnecting MQTT...");
    safeMqttDisconnect();
    delay(500); // Force re-establish - will happen in reconnect logic above
  }

  // 4) Periodic modem hard reset to clear modem memory leaks (if used)
  if (now - lastModemHardReset >= MODEM_HARD_RESET_INTERVAL) {
    Serial.println("[MOD] Periodic modem hard reset interval reached. Power-cycling modem...");
    modemPowerCycle();
    modemFullInit(); // re-establish GPRS and MQTT in the next loop cycles
    lastModemHardReset = now;
  }

  // 5) Check sensors and publish statuses (debounced)
  checkAndPublishFloatStatus();

  // 6) If modem or network shows signs of being stuck: defensive checks
  static unsigned long lastNetworkCheck = 0;
  const unsigned long NETWORK_CHECK_INTERVAL = 30UL * 1000UL; // 30s

  if (now - lastNetworkCheck >= NETWORK_CHECK_INTERVAL) {
    lastNetworkCheck = now;

    if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
      Serial.println("[NET] Network/GPRS not connected -> trying soft reconnect...");
      // try soft disconnect/connect sequence
      modem.gprsDisconnect();
      delay(400);
      ensureGprsConnected(); // if still not connected after ensureGprsConnected, it will power-cycle modem
    }
  }

  // 7) Short sleep/wait to avoid busy loop (tiny non-blocking)
  delay(50);
}
