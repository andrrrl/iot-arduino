#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT-info.h>
#include <ArduinoJson.h>

// WiFi
const char *ssid = "";           // Enter your WiFi name
const char *password = ""; // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.4.54";
const char *topic = "casa/temperatura";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

// Serial console
const int serialBaud = 115200;

StaticJsonDocument<200> docTemp;
StaticJsonDocument<200> docHum;

WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

DHT dht(DHTPIN, DHTTYPE);

uint32_t delayMS;



void readTemperature()
{

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  if (isnan(t))
  {
    Serial.println(F("Error reading temperature!"));
    client.publish("casa/temperatura", "Error reading temperature!");
  }
  else
  {

    const char *tTopic = "casa/temperatura";
    char cTemp[10];
    char *temperatureJson;
    // dtostrf(t, 5, 2, cTemp);

    docTemp["sensor"] = "temperature";
    docTemp["value"] = cTemp;

    // Generate the minified JSON and send it to the Serial port.
    // serializeJson(docTemp, Serial);

    char outTemp[128];
    int b = serializeJson(docTemp, outTemp);
    Serial.print("bytes = ");
    Serial.println(b, DEC);
    Serial.print(F("Temperature: "));
    Serial.print(t);
    Serial.println(F("°C"));

    client.publish("casa/temperatura", outTemp);
    client.subscribe(tTopic);
  }
}

void readHumidity()
{
  // Get humidity event and print its value.
  float h = dht.readHumidity();
  // dht.humidity().getEvent(&event);
  if (isnan(h))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {

    const char *hTopic = "casa/humedad";
    char cHum[10];
    char *humidityJson;
    // dtostrf(h, 5, 2, cHum);

    docHum["sensor"] = "humidity";
    docHum["value"] = cHum;

    // serializeJson(docHum, Serial);
    serializeJson(docHum, cHum);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.println(F("%"));

    client.publish(hTopic, cHum);
    client.subscribe(hTopic);
  }
}

void setup()
{
  // Set software serial baud to 115200;
  Serial.begin(serialBaud);
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  // connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected())
  {
    String client_id = "esp8266-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Public emqx mqtt broker connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  // publish and subscribe
  client.publish(topic, "Hola Colo!");
  client.subscribe(topic);

  dht.begin();
  sensor_t sensor;

  //  // Print temperature sensor info
  //  temperatureSensor(sensor);
  //
  //  // Print humidity sensor info
  //  humiditySensor(sensor);

  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void loop()
{

  client.loop();

  // Delay between measurements.
  delay(10000);

  // Read temp and humidity
  readTemperature();
  readHumidity();
}
