// https://esp32.com/viewtopic.php?t=16109

#include <WiFi.h>
#include <PubSubClient.h>

#include "driver/twai.h"

#include "config.h"

#define USB_SPEED 500000

#define RX_PIN 13
#define TX_PIN 14
#define CAN_RS 38

#define FORCE_ON 17
#define SENSE_V_DIG 8

#define BLUE_LED 10
#define YELLOW_LED 11
// Yellow LED show status of power voltage, over or under threshold
// Blue LED turns on during countdown delay until board is shut off

WiFiClient espClient;
PubSubClient mqtt(espClient);

#define WIFI_DOWN_MQTT_DOWN 1
#define WIFI_STARTING_MQTT_DOWN 2
#define WIFI_UP_MQTT_DOWN 3
#define WIFI_UP_MQTT_STARTING 4
#define WIFI_UP_MQTT_UP 5
#define WIFI_UP_MQTT_UP_AIO 6

unsigned long timeConnectStarted;
unsigned long intervalWiFi = 10000;
unsigned long intervalMQTT = 10000;
bool AIOconnected = false;

unsigned long intervalPublish = 10000;
unsigned long lastPublish = millis() - (intervalPublish / 2);

unsigned long lastShutdown;
bool enableShutdown = false;
unsigned long delayShutdown = 10000;

void setup() {
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(SENSE_V_DIG, INPUT_PULLUP);
  pinMode(FORCE_ON, OUTPUT);

  digitalWrite(FORCE_ON, HIGH);  // When kept high the board will keep running even if the power voltage drops below the threshold (indicating that the car engine is stopped and it has stopped charging the car battery)

  Serial.begin(USB_SPEED);
  Serial.setTxTimeoutMs(0);  // prevent slow timeouts
  Serial.println("Starting...");

  pinMode(CAN_RS, OUTPUT);    // INPUT (high impedance) = slope control mode, OUTPUT = see next line
  digitalWrite(CAN_RS, LOW);  // LOW = high speed mode, HIGH = low power mode (listen only)

  WiFi.mode(WIFI_STA);
  mqtt.setServer(mqtt_broker, mqtt_port);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

// Buffer for the messages with identifier 0x286, 0x373, 0x374, 0x389, 0x418
twai_message_t msg_buffer[5] = { 0 };

void loop() {
  can_read_buffered();

  // When transmission status is not P, then we can skip rest of the logic
  if (msg_buffer[4].data[0] != 80) {
    return;
  }

  shutdown_counter();

  connect_wifi_and_mqtt();

  can_interval_publish();

  mqtt.loop();
}

void can_interval_publish() {
  unsigned long now = millis();

  if (lastPublish > now) {
    lastPublish = now;
  }
  if (now - lastPublish < intervalPublish) {
    return;
  }
  lastPublish = now;

  Serial.println("Publishing to MQTT:");
  for (int i = 0; i < 5; i++) {
    if (msg_buffer[i].identifier != 0) {
      Serial.print("0x");
      Serial.print(msg_buffer[i].identifier, HEX);
      Serial.println();

      // Transmission status ignored
      if (msg_buffer[i].identifier != 0x418) {
        can_publish(&msg_buffer[i]);
      }
      msg_buffer[i].identifier = 0;
    }
  }
}

void can_publish(twai_message_t* msg) {
  char topic[64];
  sprintf(topic, "can/%x/raw", msg->identifier);

  // Raw data
  char payload[17] = { 0 };
  for (int i = 0; i < msg->data_length_code; i++) {
    sprintf(payload, "%s%02x", payload, msg->data[i]);
  }

  mqtt.publish(topic, (const uint8_t*)&payload, msg->data_length_code * 2, true);

  float current;
  float voltage;

  // Decoded values
  switch (msg->identifier) {
    case 0x286:
      sprintf(topic, "can/%x/temp", msg->identifier);
      mqtt.publish(topic, String(msg->data[3] - 40).c_str(), true);

      sprintf(topic, "can/%x/status", msg->identifier);
      mqtt.publish(topic, String((msg->data[1] & 32) == 32).c_str(), true);
      break;
    case 0x373:
      current = (((((msg->data[2] * 256.0) + msg->data[3])) - 32768)) * 0.01;
      voltage = (msg->data[4] * 256.0 + msg->data[5]) * 0.1;

      sprintf(topic, "can/%x/current", msg->identifier);
      mqtt.publish(topic, String(current).c_str(), true);

      sprintf(topic, "can/%x/energy", msg->identifier);
      mqtt.publish(topic, String(voltage).c_str(), true);

      sprintf(topic, "can/%x/power", msg->identifier);
      mqtt.publish(topic, String((voltage * current) * 0.001).c_str(), true);
      break;
    case 0x374:
      sprintf(topic, "can/%x/capacity", msg->identifier);
      mqtt.publish(topic, String((msg->data[1] - 10) * 0.5).c_str(), true);
      break;
    case 0x389:
      sprintf(topic, "can/%x/energy", msg->identifier);
      mqtt.publish(topic, String(msg->data[1] * 1.0).c_str(), true);

      sprintf(topic, "can/%x/current", msg->identifier);
      mqtt.publish(topic, String(msg->data[6] * 0.1).c_str(), true);
      break;
  }
}

void can_read_buffered() {
  twai_message_t msg;

  if (twai_receive(&msg, 100) != ESP_OK) {
    return;
  }

  switch (msg.identifier) {
    case 0x286:
      msg_buffer[0] = msg;  // Charger status / inverter temperature
      break;
    case 0x373:
      msg_buffer[1] = msg;  // Main Battery volt and current
      break;
    case 0x374:
      msg_buffer[2] = msg;  // Main Battery Soc
      break;
    case 0x389:
      msg_buffer[3] = msg;  // Charger voltage and current
      break;
    case 0x418:
      msg_buffer[4] = msg;  // Transmissin state
      break;
    default:
      return;
  }

#ifdef DEBUG
  Serial.print("0x");
  Serial.print(msg.identifier, HEX);
  Serial.print("\t");
  Serial.print(msg.extd);
  Serial.print("\t");
  Serial.print(msg.rtr);
  Serial.print("\t");
  Serial.print(msg.data_length_code);

  for (int i = 0; i < msg.data_length_code; i++) {
    Serial.print("\t0x");
    if (msg.data[i] <= 0x0F) {
      Serial.print(0);
    }
    Serial.print(msg.data[i], HEX);
  }
  Serial.println();
#endif
}

void shutdown_counter() {
  unsigned long now = millis();

  if (!digitalRead(SENSE_V_DIG)) {
    if (enableShutdown == false) {
      enableShutdown = true;
      lastShutdown = now;
      digitalWrite(BLUE_LED, HIGH);
    }
  } else {
    enableShutdown = false;
    digitalWrite(BLUE_LED, LOW);
  }
  if (enableShutdown && now - lastShutdown >= delayShutdown) {
    digitalWrite(FORCE_ON, LOW);
  }
  digitalWrite(YELLOW_LED, digitalRead(SENSE_V_DIG));
}

void connect_wifi_and_mqtt() {
  // The first time the function runs, the WiFi and MQTT broker
  // are disconnected; therefore set the state machine's initial state
  static byte stateConnection = WIFI_DOWN_MQTT_DOWN;

  // Next, check if the previously successful WiFi and MQTT broker connection was dropped
  if (WiFi.status() == WL_DISCONNECTED && stateConnection == WIFI_UP_MQTT_UP_AIO) {
    Serial.println("Restart WLAN connection");
    stateConnection = WIFI_DOWN_MQTT_DOWN;
    AIOconnected = false;
  }

  switch (stateConnection) {
    // If there is no WiFi  connection
    case WIFI_DOWN_MQTT_DOWN:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Start WiFi connection");
        WiFi.begin(ssid, password);
        timeConnectStarted = millis();
        stateConnection = WIFI_STARTING_MQTT_DOWN;
      }
      break;

    // If the WiFi connection was started
    case WIFI_STARTING_MQTT_DOWN:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to the WiFi network");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        stateConnection = WIFI_UP_MQTT_DOWN;
      } else if (millis() - timeConnectStarted >= intervalWiFi) {
        Serial.println("Retry WiFi connection");
        WiFi.disconnect();
        stateConnection = WIFI_DOWN_MQTT_DOWN;
      }
      break;

    // If the WiFi connection was established
    case WIFI_UP_MQTT_DOWN:
      if ((WiFi.status() == WL_CONNECTED) && !mqtt.connected()) {
        Serial.println("WiFi connected. Start MQTT connection");
        timeConnectStarted = millis();
        stateConnection = WIFI_UP_MQTT_STARTING;
      }
      break;

    // If the MQTT broker connection was started
    case WIFI_UP_MQTT_STARTING:
      if (mqtt.connect("esp32-client-can-bus")) {
        stateConnection = WIFI_UP_MQTT_UP;
      } else if (millis() - timeConnectStarted >= intervalMQTT) {
        Serial.println("Retry MQTT connection");
        stateConnection = WIFI_UP_MQTT_DOWN;
      }
      break;

    // If both the WiFi and MQTT broker connections were established
    case WIFI_UP_MQTT_UP:
      Serial.println("WiFi and MQTT connected");

      // Toggle flag to enable publishing and subscribing
      AIOconnected = true;

      stateConnection = WIFI_UP_MQTT_UP_AIO;
      break;

    case WIFI_UP_MQTT_UP_AIO:
      // The main loop
      break;
  }
}
