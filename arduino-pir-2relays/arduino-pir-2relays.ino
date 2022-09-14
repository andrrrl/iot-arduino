
// Relay 1
const int RELAY_PIN_1 = 10;

// Relay 2
const int RELAY_PIN_2 = 11;

// PIR Sensor
const int PIR_PIN = 8;

// LED
const int LED_PIN = 13;

int pirState = LOW; // de inicio no hay movimiento
int pirValue = 0; // estado del pin

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_PIN_1, OUTPUT);
  // pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  
  pirValue = digitalRead(PIR_PIN);
 
  if (pirValue == HIGH) {
    if (pirState == LOW) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(RELAY_PIN_1, LOW);
      // digitalWrite(RELAY_PIN_2, LOW);
        Serial.println("Sensor activado, epa Colo.");
        pirState = HIGH;
      }
  } else {
    if (pirState == HIGH) {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(RELAY_PIN_1, HIGH);
      // digitalWrite(RELAY_PIN_2, HIGH);
      Serial.println("Sensor parado, ojo Colo.");
      pirState = LOW;
    }
  }
}
