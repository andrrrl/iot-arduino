#include <Arduino.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "Env.h"  // Comillas para directorio local
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// NTP
const long utcOffsetArgentinaInSeconds = -10800;
String daysOfTheWeek[7] = { "Domingo", "Lunes", "Martes", "Miércoles", "Jueves", "Viernes", "Sábado" };
String months[12] = { "Enero", "Febrero", "Marzo", "Abril", "Mayi", "Junio", "Julio", "Agosti", "Septiembre", "Octubre", "Noviembre", "Diciembre" };

unsigned long previousMinute = 0;

// 1. MQTT PubSub

// If server is down, work in "offline mode"
bool noMqttMode = false;

// Wi-Fi config
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 2. Define NTP Client to get time
WiFiUDP espClientUDP;
NTPClient timeClient(espClientUDP, "pool.ntp.org", utcOffsetArgentinaInSeconds);

// 3. SENSORS
// Digital pin connected to the DS18B20 temperature sensor
const int oneWireBus = D4;  // D4

// Setup a oneWire instance to communicate with any DS18B20
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to DS18B20
DallasTemperature ds18b20(&oneWire);

// Setup DHT-11
// Digital pin connected to the DHT temprerature/humidity sensor
const int dhtPin = D6;
const uint8_t dhtType = DHT11;
DHT dht(dhtPin, dhtType);

// Analog sensor pin connected to Soil moisture sensor
// const int moistureSensorPin = A0;
int dryValue = 570;
int wetValue = 0;
int friendlyDryValue = 0;
int friendlyWetValue = 100;

// PIR Sensor
const int PIR_PIN = D0;
int pirState = LOW;  // de inicio no hay movimiento
int pirValue = 0;    // estado del pin

// LED
const int LED_PIN = D2;
bool isLedOn = false;

// 5. SERIAL CONSOLE
const int serialBaud = 115200;

// 6. Prepare JSON data
JsonDocument docTemperaturePrecise;
JsonDocument docMoisture;
JsonDocument docTemperature;
JsonDocument docHumidity;
JsonDocument docMqttCallback;

void printValue(const char* textStart, int value, const char* textEnd) {
  Serial.print(textStart);
  Serial.print(value);
  Serial.println(textEnd);
}

void connectWifi() {
  Serial.print("[INFO] Conectando a red Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }
  Serial.print("\n");
  Serial.println("[INFO] Conectado a la red Wi-Fi!");
  Serial.print("[INFO] Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void toggleExternalLed(bool state) {
  digitalWrite(LED_PIN, state ? HIGH : LOW);
}

void blinkExternalLedTimes(int repeats, int time = 1000) {
  for (unsigned int i = 0; i < repeats; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(time);
    digitalWrite(LED_PIN, LOW);
    delay(time);
  }
}

void subscribeToMqttTopics() {
  Serial.println("[INFO] Suscribiendo a los topics...");
  boolean preciseTempSub = mqttClient.subscribe(MQTT_TOPIC_HOUSE_TEMP_DS18B20);
  boolean aproxTempSub = mqttClient.subscribe(MQTT_TOPIC_HOUSE_TEMP_DHT11);
  boolean AmbientHumiditySub = mqttClient.subscribe(MQTT_TOPIC_HOUSE_HUMIDITY);
  // boolean soilMoistureSub = mqttClient.subscribe(MQTT_TOPIC_HOUSE_SOIL_MOISTURE);

  if (preciseTempSub && aproxTempSub && AmbientHumiditySub) {
    Serial.println("[INFO] Suscripción a topics exitosa...");
  } else {
    Serial.println("[ERROR] Falló la suscripción...");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Nuevo mensaje en el topic ");
  Serial.print(topic);
  Serial.println(":");

  String content = "";
  for (unsigned int i = 0; i < length; i++) {
    content.concat((char)payload[i]);
  }
  Serial.print(content);
  Serial.println();

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docMqttCallback, content);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  const char* mqttCallbackType = docMqttCallback["type"].as<const char*>();
  const bool mqttCallbackState = docMqttCallback["state"].as<const bool>();

  // Check if mqttCallbackType is not NULL
  if (mqttCallbackType != NULL) {

    Serial.println("[DEBUG] Inspecting docMqttCallback:");
    serializeJsonPretty(docMqttCallback, Serial);

    // Execute instruction:
    if (strcmp(mqttCallbackType, "SWITCH") == 0) {
      Serial.printf("[INFO] %s\n", mqttCallbackType);
      toggleExternalLed(mqttCallbackState);
    } else {
      Serial.println("[WARN] Comando desconocido.");
    }
  }
}

void connectMqtt() {
  Serial.print("[INFO] Conectando al broker MQTT...");

  // Server
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  // Process messages from server
  mqttClient.setCallback(mqttCallback);

  String client_id = "node-mcu-client-" + String(WiFi.macAddress());

  int connectRetryCount = 0;

  if (noMqttMode == false) {
    while (!mqttClient.connected()) {
      Serial.print(".");

      if (connectRetryCount == 5) {
        noMqttMode = true;
        break;
      }

      if (mqttClient.connect(client_id.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.print("\n");
        Serial.printf("[INFO] Conectado al broker MQTT %s!\n", client_id.c_str());
      } else {
        Serial.print("[ERROR] Falló la conexión, state: ");
        Serial.println(mqttClient.state());
        connectRetryCount++;
      }
      delay(2000);
    }
  }
}

// DHT-11
void readTemperature() {

  // Read aproximate temperature (DHT-11 is not very precise)
  float temperature = dht.readTemperature();

  if (noMqttMode == false) {
    // Set JSON document
    docTemperature["sensor"] = "DHT-11 Temperatura";
    docTemperature["value"] = temperature;

    // Serialize JSON, save in outTempJSON variable
    char outTempJSON[128];
    serializeJson(docTemperature, outTempJSON);

    // Send JSON to MQTT server
    mqttClient.publish(MQTT_TOPIC_HOUSE_TEMP_DHT11, outTempJSON);
  }

  printValue("[SENSOR] Temperatura aproximada: ", temperature, "°C");
}

// DHT-11
void readHumidity() {

  // Read ambient humidity
  float humidity = dht.readHumidity();

  if (noMqttMode == false) {

    // Set JSON document
    docHumidity["sensor"] = "DHT-11 Humedad ambiente";
    docHumidity["value"] = humidity;

    // Serialize JSON, save in outHumJSON variable
    char outHumJSON[128];
    serializeJson(docHumidity, outHumJSON);

    // Send JSON to MQTT server
    mqttClient.publish(MQTT_TOPIC_HOUSE_HUMIDITY, outHumJSON);
  }

  // Print sensor reading to seial console
  printValue("[SENSOR] Humedad ambiente: ", humidity, "%");

  if (humidity < 30) {
    blinkExternalLedTimes(5);
  }
}

// DS18B20
void readTemperaturePrecise() {

  // Read temperature
  ds18b20.requestTemperatures();
  float temperatureC = ds18b20.getTempCByIndex(0);

  if (noMqttMode == false) {
    // Set JSON document
    docTemperaturePrecise["sensor"] = "DS18B20 Temperatura";
    docTemperaturePrecise["value"] = temperatureC;

    // Serialize JSON, save in outTempJSON variable
    char outTempJSON[128];
    serializeJson(docTemperaturePrecise, outTempJSON);

    // Send JSON to MQTT server
    mqttClient.publish(MQTT_TOPIC_HOUSE_TEMP_DS18B20, outTempJSON);
  }

  // Print sensor reading to seial console
  printValue("[SENSOR] Temperatura precisa: ", temperatureC, "°C");
}

void getTime() {
  timeClient.update();

  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
  //Serial.println(timeClient.getFormattedTime());
}


void detectMovement() {

  pirValue = digitalRead(PIR_PIN);
  Serial.println(pirValue);

  if (pirValue == HIGH) {
    if (pirState == LOW) {
      Serial.println("[SENSOR] Sensor activado, epa es Colo?");
      toggleExternalLed(true);
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH) {
      Serial.println("[SENSOR] Sensor parado, none ta Colo?");
      toggleExternalLed(false);
      pirState = LOW;
    }
  }
}

void turnOnBuiltInLed() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
}

// Runs once
void setup() {

  // Serial Monitor
  Serial.begin(serialBaud);

  // ⌜⌝⌞⌟⌈⌉⎣⎦⌊⌋⎡⎤⎢⎥
  Serial.println();
  Serial.println("⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺🐟⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼");
  Serial.println("⎻⎺⎻⎼⎽⎼⎻⎺⎻🐟⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻");
  Serial.println("⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼⎽⎼⎻⎺⎻⎼🐟⎼⎻⎺⎻⎼⎽⎼⎻⎺");
  Serial.println("[INFO] ¡BIENVENIDOS A COLORINCHE IOT!");
  Serial.println("\n[INFO] Iniciando dispositivo...");

  // NTP Client
  timeClient.begin();

  // Wi-Fi
  connectWifi();

  // MQTT
  if (noMqttMode == false) {
    connectMqtt();
    subscribeToMqttTopics();
  }

  // DS18B20 sensor
  ds18b20.begin();

  // DHT-11 sensor
  dht.begin();

  // PIR sensor
  pinMode(PIR_PIN, INPUT);  // Set PIR pin as input

  // LEDs
  pinMode(LED_PIN, OUTPUT);
}


// Run
void loop() {

  // Update time
  timeClient.update();

  // Listen for messages
  mqttClient.loop();

  // Detect movement continuously
  detectMovement();

  // Check if a minute has passed
  unsigned long currentMinute = timeClient.getMinutes();
  if (currentMinute != previousMinute) {
    previousMinute = currentMinute;

    // Blink NodeMCU internal LED once, just because
    // turnOnBuiltInLed();

    // Print date and time
    Serial.print("[INFO] Nueva lectura a las ");
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" horas.\n");
    Serial.println("[INFO] Leyendo datos de sensores... ");

    // Read sensor data
    readTemperaturePrecise();
    readTemperature();
    readHumidity();

    // Moiste sensore is broken -_-
    // readMoisture();
  }

  delay(1000);
}