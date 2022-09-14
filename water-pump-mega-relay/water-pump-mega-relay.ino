// Relay
// const int RELAY_PIN = 13;

// Water Pump
const int PUMP_PIN = 13;

void setup() {
  // pinMode(RELAY_PIN, OUTPUT);
  // // pinMode(RELAY_PIN, OUTPUT);
  // pinMode(LED_PIN, OUTPUT);

  pinMode(PUMP_PIN, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  
  Serial.println("Encender bomba de agua");
      
  digitalWrite(PUMP_PIN, HIGH);

  delay(10000);

  Serial.println("Apagar bomba de agua");

  digitalWrite(PUMP_PIN, LOW);

  delay(10000);

}
