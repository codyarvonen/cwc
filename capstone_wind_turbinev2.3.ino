/**
   capstone_wind_turbinev2.ino

   This contains all the code required for the competition

   Written for the Collegiate Wind Competition
   by the 2022 team 47

   Contributors: Spencer Truman
   Revision: 2.0
   Last uploaded to box: 7 FEB 2022

   Encoder Library by Paul Stroffregen
   It can find it by searching in the library manager for the Encoder librabry
   and then scrolling down for "Encoder" by "Paul Stroffregen"
   http://www.pjrc.com/teensy/td_libs_Encoder.html

   Revision History:
   Rev 1.0 - Initial code, basic arduino functionality and rough outline of states
   Rev 1.1 - Copy functions from last years code
   Rev 1.2 - Implement the ebrake functionality
   Rev 1.3 - Implement the linear acuator for pitch control
   Rev 1.4 - Implement the mechanical brake servo motor
   Rev 1.5 - Implement the rpm sensor
   Rev 2.0 - Updates for new encoder and eddy brake
   Rev 2.1 - Rewrote linear actuator control function
   Rev 2.2 - Updated cut in pitch angle and cleaned up code
   Rev 2.3 - Removed mechanical brake
*/

#include <Servo.h>
#include <Encoder.h>

//   Change the two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(1, 2);

/* PINS */
#define ISOLATOR_PIN 7
#define ESTOP_SWITCH_PIN  8

//Used for the encoder
long oldPosition  = -999;
long oldtime = 0;
float dx;
float dt;
float cps;
float rps;
float old_rps;
float delta_rps;

//used in the pitch control function
Servo pitchControl; // This is the linear actuator to control pitch angle
float currentPitchAngle;
int pos = 0;    // variable to store the servo position
int pitch_angle = 0;
int current_angle = 0;


enum states_t
{
  Default_st,
  Startup_st,
  Optimize_st,
} currentState;

void setup() {
  pinMode(ISOLATOR_PIN, OUTPUT);
  pinMode(ESTOP_SWITCH_PIN, INPUT);
  digitalWrite(ISOLATOR_PIN, LOW);
  Serial.begin(9600);
  pitchControl.attach(12);
}


void loop() {
  if (Serial.available() > 0) {
    pitch_angle = Serial.read();
  }
  encoder();

  float estopReading = digitalRead(ESTOP_SWITCH_PIN);
  Serial.println(currentState);
  switch (currentState)
  {
    case Default_st:
      if (estopReading == HIGH) {
        currentState = Startup_st;
      }
      digitalWrite(ISOLATOR_PIN, LOW);
      currentPitchAngle = 0;
      pitch_angle = 50;
      set_pitch();
      delay(50);
      break;


    case Startup_st:
      if (estopReading == LOW) {
        currentState = Default_st;
      }
      digitalWrite(ISOLATOR_PIN, HIGH);  // COMMENT OUT THIS LINE OF CODE FOR THE COMPETITION
      pitch_angle = 70;
      if (rps > 4 && rps < 10000) {
        currentState = Optimize_st;
      }
      break;


    case Optimize_st:
      if (estopReading == LOW) {
        currentState = Default_st;
      }
      digitalWrite(ISOLATOR_PIN, HIGH);
      pitch_angle = 90;
      set_pitch();
      if (rps < 4 || rps > 10000) {
        currentState = Startup_st;
      }
      break;
  }
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
    Serial.println(rps);
  }
}

void set_pitch() {
  if (pitch_angle < current_angle) {
    //Serial.println("contracting to:");
    for (pos = current_angle; pos >= pitch_angle; pos -= 1) {
      pitchControl.write(pos);
      delay(15);
    }
    current_angle = pitch_angle;
  }
  if (pitch_angle > current_angle) {
    //Serial.println("extending to:");
    for (pos = current_angle; pos <= pitch_angle; pos += 1) {
      pitchControl.write(pos);
      delay(15);
    }
    current_angle = pitch_angle;
  }
  delay(15);
}
