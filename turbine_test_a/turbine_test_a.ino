// https://github.com/arduino-libraries/Servo
#include <Servo.h>
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2

#define ACTUATOR_PIN 12
#define BRAKE_SERVO_PIN 9

const int ENGAGED_BRAKE_ANGLE = 95;
const int DISENGAGED_BRAKE_ANGLE = 120;
const int ACTUATOR_MAX = 100;
const int ACTUATOR_MIN = 40;

bool brakeIsEngaged = false;

Servo pitchControl;
Servo brakeControl;

//   Change the two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);

//Used for the encoder
long oldPosition  = -999;
long oldtime = 0;
float dx;
float dt;
float cps;
float rps;
float old_rps;
float delta_rps;

void setup() {
    Serial.begin(9600);
    Serial.write("Beginning test! \n");
    pitchControl.write(80);
    pitchControl.attach(ACTUATOR_PIN);
    brakeControl.write(DISENGAGED_BRAKE_ANGLE);
    brakeControl.attach(BRAKE_SERVO_PIN);
}

unsigned int integerValue = 0;
char incomingByte;
int pitch;

void loop() {

  //HANDLE COMMANDS FROM SERIAL MONITOR FOR PITCH AND BRAKE
  if (Serial.available() > 0) {   // something came across serial
    integerValue = 0;      // throw away previous integerValue
    while(1) {        // force into a loop until '\n' is received
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
      if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
      integerValue *= 10;  // shift left 1 decimal place
      // convert ASCII to integer, add, and shift left 1 decimal place
      integerValue = ((incomingByte - 48) + integerValue);
    }
    // Do something with the value
    pitch = integerValue;
    if (pitchControl.attached()) {
      if (ACTUATOR_MIN <= pitch && pitch <= ACTUATOR_MAX){
        Serial.print("Setting pitch to ");
        Serial.println(pitch); 
        set_pitch(pitch);  
      } else if (pitch == 19383){ //the decimal representation of 'BRAKE'
        if (!brakeIsEngaged){
          Serial.println("Engaging Brake");
          engageBrake();
          brakeIsEngaged = true;
        } else {
          Serial.println("Disengaging Brake");
          disengageBrake();
          brakeIsEngaged = false;
        }
      } else {
        Serial.println("Invalid Input");
      }
    } else {
      Serial.println("Actuator is not attached.");
    }
  }

  //HANDLE ENCODER
  delay(1000);
  encoder();
}

void set_pitch(int pitch_angle) {
    int current_angle = pitchControl.read();
    if (pitch_angle < current_angle) {
        for (int pos = current_angle; pos >= pitch_angle; pos--) {
            pitchControl.write(pos);
            delay(15);
        }
    }
    if (pitch_angle > current_angle) {
        for (int pos = current_angle; pos <= pitch_angle; pos++) {
            pitchControl.write(pos);
            delay(15);
        }
    }
    delay(15);
}

void engageBrake() {
  brakeControl.write(ENGAGED_BRAKE_ANGLE);
}

void disengageBrake(){
  brakeControl.write(DISENGAGED_BRAKE_ANGLE);
}

void encoder() {
  long newPosition = myEnc.read();
  
  if (newPosition != oldPosition) {
    long newtime = micros();
    dx = (newPosition - oldPosition);
    dt = newtime - oldtime;
    oldPosition = newPosition;
    oldtime = newtime;
    cps = dx / (dt / 1000000);
    rps = cps / 2048;
    delta_rps = rps - old_rps;
    old_rps = rps;
    Serial.println(rps*60);
  }
}
