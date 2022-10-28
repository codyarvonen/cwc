
#define TRIGGER_PIN 13

void setup() {

    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    
    Serial.begin(9600);
    Serial.write("Beginning test! \n");

}

void loop() {
  
    digitalWrite(TRIGGER_PIN, LOW);
    delay(1000);
    digitalWrite(TRIGGER_PIN, HIGH);
    delay(1000);

}
