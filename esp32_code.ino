#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

#define RO_PIN 16
#define DI_PIN 17
#define DE_PIN 18
#define RE_PIN 19
#define RS485_BAUD 9600

const char* ssid = "IMS-ELECTRIC";
const char* password = "Not_$chneider786!";
const char* mqtt_server = "172.16.10.82";
const char* mqtt_topic = "/IMS/Data";

uint16_t data_registers[] = {3019, 3020, 3021, 3030, 3032};
const int NUM_REGISTERS = sizeof(data_registers) / sizeof(data_registers[0]);
uint16_t registers[NUM_REGISTERS];

ModbusMaster node;
WiFiClient wifiClient;
AsyncMqttClient mqttClient;
unsigned long lastMillis = 0;
const unsigned long interval = 500; 

void modbusPreTransmission() {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(RE_PIN, HIGH);
}

void modbusPostTransmission() {
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}

void connectToWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void connectToMqtt() {
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT broker");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT broker");
}

void setup() {
  Serial.begin(115200);
  pinMode(DE_PIN, OUTPUT);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);

  Serial2.begin(RS485_BAUD, SERIAL_8E1, RO_PIN, DI_PIN);
  Serial2.setTimeout(2000);
  node.begin(1, Serial2);
  node.preTransmission(modbusPreTransmission);
  node.postTransmission(modbusPostTransmission);

  connectToWifi();

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(mqtt_server, 1883);

  connectToMqtt();
}

void loop() {
  if (millis() - lastMillis >= interval) {
    lastMillis = millis();
    AsyncMqttClientInternals::DefaultMessageProperties msgProps;
    DynamicJsonDocument doc(1024);
    JsonObject payload = doc.createNestedObject("payload");
    for (int i = 0; i < NUM_REGISTERS; i++) {
      uint16_t value = 0;
      node.readHoldingRegisters(data_registers[i], 1);
      value = node.getResponseBuffer(0);
      payload[String(data_registers[i])] = value;
    }
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    mqttClient.publish(mqtt_topic, 0, false, jsonBuffer, msgProps);
  }

  if (WiFi.status() != WL_CONNECTED) {
    connectToWifi();
  }

  if (!mqttClient.connected()) {
    connectToMqtt();
  }

  mqttClient.loop();
}

