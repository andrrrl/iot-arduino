// Relay
const int RELAY_PIN_PUMP = 11;

void setup() {

  pinMode(RELAY_PIN_PUMP, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  
  Serial.println("Encender bomba de agua");
      
  digitalWrite(RELAY_PIN_PUMP, HIGH);

  delay(1000);

  Serial.println("Apagar bomba de agua");

  digitalWrite(RELAY_PIN_PUMP, LOW);

  delay(100000);

}
