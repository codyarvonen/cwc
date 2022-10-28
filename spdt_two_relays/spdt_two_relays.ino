
#define TRIGGER_PIN_1 13
#define TRIGGER_PIN_2 11

void setup() {

    pinMode(TRIGGER_PIN_1, OUTPUT);
    digitalWrite(TRIGGER_PIN_1, LOW);
    pinMode(TRIGGER_PIN_2, OUTPUT);
    digitalWrite(TRIGGER_PIN_2, LOW);
    
    Serial.begin(9600);
    Serial.write("Beginning test! \n");

}

void loop() {
  
    digitalWrite(TRIGGER_PIN_2, LOW);
    delay(500);
    digitalWrite(TRIGGER_PIN_1, HIGH);
    delay(1000);
    digitalWrite(TRIGGER_PIN_1, LOW);
    delay(500);
    digitalWrite(TRIGGER_PIN_2, HIGH);
    delay(1000);

}
