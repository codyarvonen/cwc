#include <Servo.h>
Servo myServo;
#define SERVO_PIN 9
const int ENGAGED_BRAKE_ANGLE = 90;
const int DISENGAGED_BRAKE_ANGLE = 110;
const int frequency = 1;
int timeDelay;
void setup() {
    myServo.attach(SERVO_PIN);
    timeDelay = 1000/frequency;
}
void loop() {
    pulseBraking(timeDelay);
}
void engageBrake() {
  myServo.write(ENGAGED_BRAKE_ANGLE);
}
void disengageBrake(){
  myServo.write(DISENGAGED_BRAKE_ANGLE);
}
//The Large Servo cannot handle high frequency pulses,
//For consistent angles use 1 second time delay (frequency 1)
void pulseBraking(int timeDelay){
  engageBrake();
  delay(timeDelay);
  disengageBrake();
  delay(timeDelay);
}