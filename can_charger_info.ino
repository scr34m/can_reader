// https://esp32.com/viewtopic.php?t=16109

#include <WiFi.h>
#include <PubSubClient.h>

#include "driver/twai.h"

#include "config.h"

#define USB_SPEED 500000

#define RX_PIN 13
#define TX_PIN 14
#define CAN_RS 38

#define SECONDS_TO_STAY_ON 5

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

#ifdef DEBUG
twai_message_t d_msg;
#endif

void setup() {
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(SENSE_V_DIG, INPUT_PULLUP);
  pinMode(FORCE_ON, OUTPUT);

  digitalWrite(FORCE_ON, HIGH);  // When kept high the board will keep running even if the power voltage drops below the threshold (indicating that the car engine is stopped and it has stopped charging the car battery)

  Serial.begin(USB_SPEED);
  Serial.setTxTimeoutMs(0);  // prevent slow timeouts

  pinMode(CAN_RS, OUTPUT);    // INPUT (high impedance) = slope control mode, OUTPUT = see next line
  digitalWrite(CAN_RS, LOW);  // LOW = high speed mode, HIGH = low power mode (listen only)

  WiFi.mode(WIFI_STA);
  mqtt.setServer(mqtt_broker, mqtt_port);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

#ifdef DEBUG
  d_msg.identifier = 0x111;
  d_msg.data_length_code = 3;
  d_msg.data[0] = 0x10;
  d_msg.data[1] = 0x20;
  d_msg.data[2] = 0x30;
#endif
}

void loop() {
  connect_wifi_and_mqtt();
  // shutdown_counter();
  if (!AIOconnected) {
    delay(100);
  } else {
    can_read();
    mqtt.loop();

#ifdef DEBUG
    d_msg.data[0] += 1;
    can_publish(&d_msg);
    delay(1000);
#endif
  }
}

void can_publish(twai_message_t* msg) {
  char topic[9];
  sprintf(topic, "can/%x", msg->identifier);

  char payload[17] = { 0 };
  for (int i = 0; i < msg->data_length_code; i++) {
    sprintf(payload, "%s%02x", payload, msg->data[i]);
  }

  mqtt.publish(topic, (const uint8_t*)&payload, msg->data_length_code * 2, true);
}

void can_read() {
  twai_message_t msg;

  if (twai_receive(&msg, 0) != ESP_OK) {
    return;
  }

  if (
    msg.identifier != 0x286 && msg.identifier != 0x697 && msg.identifier != 0x373 && msg.identifier != 0x374 && msg.identifier != 0x389 && msg.identifier != 0x6E1 && msg.identifier != 0x6E2 && msg.identifier != 0x6E3 && msg.identifier != 0x6E4) {
    return;
  }

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

  can_publish(&msg);
}

void shutdown_counter() {
  static unsigned long timestamp;
  static bool countdown = false;
  unsigned long now = millis();

  if (!digitalRead(SENSE_V_DIG)) {
    if (countdown == false) {
      countdown = true;
      timestamp = now;
      digitalWrite(BLUE_LED, HIGH);
    }
  } else {
    countdown = false;
    digitalWrite(BLUE_LED, LOW);
  }
  if (countdown && now - timestamp >= SECONDS_TO_STAY_ON * 1000) {
    digitalWrite(FORCE_ON, LOW);
  }
  digitalWrite(YELLOW_LED, digitalRead(SENSE_V_DIG));
}

void connect_wifi_and_mqtt() {
  // The first time the function runs, the WLAN router and MQTT broker
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
