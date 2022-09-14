#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ArduinoJson.h>

// 1. WI-FI
// Wi-Fi config
const char *ssid = "XTRMNTR";
const char *password = "juanitaladel8";
WiFiClient espClient;
PubSubClient client(espClient);

// 2. MQTT
// Broker
const char *mqtt_broker = "192.168.4.54";
const char *mqtt_username = "colorina";
const char *mqtt_password = "colorinche7";
const int mqtt_port = 1883;

// Topics
// DS18B20
const char *temperaturePreciseTopic = "casa/temperatura-precisa";
// DHT-11 Temperature
const char *temperatureTopic = "casa/temperatura";
// DHT-11 Ambient Humidity
const char *humidityTopic = "casa/humedad-ambiente";
// Moisture sensor
const char *moistureTopic = "casa/humedad-suelo";

// 4. SENSORS
// GPIO where the DS18B20 is connected to
const int oneWireBus = 4; 

// Digital pin connected to the DHT sensor 
const int dhtPin = D6;

// Analog sensor pin connected to Soil moisture sensor
const int moistureSensorPin = A0;  

// Setup a oneWire instance to communicate with any DS18B20
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to DS18B20
DallasTemperature ds18b20(&oneWire);

// Setup DHT-11
const uint8_t dhtType = DHT11;
DHT dht(dhtPin, dhtType);

// 4. SERIAL CONSOLE
const int serialBaud = 115200;

// 5. JSON data
StaticJsonDocument<256> docTemperaturePrecise;
StaticJsonDocument<256> docMoisture;
StaticJsonDocument<256> docTemperature;
StaticJsonDocument<256> docHumidity;


// Runs once
void setup() {

  // Wi-Fi
  connectWifi();

  // MQTT
  connectMqtt();

  // Serial Monitor
  Serial.begin(serialBaud);

  // DS18B20 sensor
  ds18b20.begin();
  
  // DHT-11 sensor
  dht.begin();
}


// Runs
void loop() {
  readTemperaturePrecise();
  readMoisture();
  readTemperature();
  readHumidity();
  
  // Wait before new readings
  delay(10000);
}

void connectWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to the WiFi network!");
}

void connectMqtt() {
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "node-mcu-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Public emqx mqtt broker connected!");
      } else {
          Serial.print("Failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (unsigned int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

// DS18B20
void readTemperaturePrecise() {

  // Read temperature
  ds18b20.requestTemperatures(); 
  float temperatureC = ds18b20.getTempCByIndex(0);

  // Set JSON document
  docTemperaturePrecise["sensor"] = "DS18B20";
  docTemperaturePrecise["value"] = temperatureC;
  
  // Serialize JSON, save in outTempJSON variable
  char outTempJSON[128];
  serializeJson(docTemperaturePrecise, outTempJSON);

  // Send JSON to MQTT server
  client.publish(temperaturePreciseTopic, outTempJSON);
  client.subscribe(temperaturePreciseTopic);

  // Print sensor reading to seial console
  printSerial("Precise Temperature: ", temperatureC, "°C");
}

// DHT-11
void readTemperature() {

  // Read temperature
  float temperature = dht.readTemperature();

  // Set JSON document
  docTemperature["sensor"] = "DHT-11";
  docTemperature["value"] = temperature;

  // Serialize JSON, save in outMoistJSON variable
  char outTempJSON[128];
  serializeJson(docTemperature, outTempJSON);

  // Send JSON to MQTT server
  client.publish(temperatureTopic, outTempJSON);
  client.subscribe(temperatureTopic);
  
  // Print sensor reading to seial console
  printSerial("Temperature: ", temperature, "%");

}

// DHT-11
void readHumidity() {

  // Read ambient humidity
  float humidity = dht.readHumidity();

  // Set JSON document
  docHumidity["sensor"] = "DHT-11";
  docHumidity["value"] = humidity;

  // Serialize JSON, save in outMoistJSON variable
  char outHumJSON[128];
  serializeJson(docHumidity, outHumJSON);
  
  // Send JSON to MQTT server
  client.publish(humidityTopic, outHumJSON);
  client.subscribe(humidityTopic);

  // Print sensor reading to seial console
  printSerial("Humidity: ", humidity, "%");
}

// Soil moisture sensor
void readMoisture() {

  // Read soil moisture
  // float moisturePercentage = ( 100.00 - ( (analogRead(moistureSensorPin)/1023.00) * 100.00 ) );
  float moisturePercentage = 442.00 - analogRead(moistureSensorPin);

  // Set JSON document
  docMoisture["sensor"] = "MoistureSensor";
  docMoisture["value"] = moisturePercentage;

  // Serialize JSON, save in outMoistJSON variable
  char outMoistJSON[128];
  serializeJson(docMoisture, outMoistJSON);

  // Send JSON to MQTT server
  client.publish(moistureTopic, outMoistJSON);
  client.subscribe(moistureTopic);

  // Print sensor reading to seial console
  printSerial("Soil Moisture: ", moisturePercentage, "%");
  
}

void printSerial(char* textStart, int value, char* textEnd) {
  Serial.print(textStart);
  Serial.print(value);
  Serial.println(textEnd);
}