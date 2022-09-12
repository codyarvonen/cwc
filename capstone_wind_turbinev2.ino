/**
   WindControl.ino

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
*/

#include <Servo.h>
#include <Encoder.h>
Servo pitchControl; // This is the linear actuator to control pitch angle
Servo brakeservo; // This is the servo motor that controls the mechanical brake

//   Change the two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);

/* PINS */
#define ESTOP_SWITCH_PIN  8 //
//#define PITCH_CONTROL_PIN  9 //controls the linear actuator for the pitch control
#define servoPin  10 //controls the servo motor for the mechanical brake

//define constants to be used throughout the code which i think are not unneccessary
#define MIN_PITCH_MICROSECONDS 1200 //the minimal value for the linear actuator
#define MAX_PITCH_MICROSECONDS 1820 //the maximum value for the linear actuator

//Used for the encoder
long oldPosition  = -999;
long oldtime = 0;
float dx;
float dt;
float cps;
float rps;

//used in the mechanical brake functions
int angle{0};
int max_angle = 125;

//used in the pitch control function
float currentPitchAngle;
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int pitch_angle = 0;
int current_angle = 0;

/**
  Define the different states
   Default_st: The ebrake State
   Startup_st: This state should focus on low cut in speed
   Optimize_st: This state will optimize load and pitch angle for windspeed versus power output

  Default_st
   should default to this state when ebrake is pressed or power is cut
   activate mechanical brake
   pitch the blades parallel to wind direction

  Startup_st (potentially could be better to have a startup_st and resume_st?)
   Enter this state when powered on or ebrake is released
   release mechanical brake
   Pitch blades to optimal angle for low cut in speed
   begin reading for turbin rotation

  Optimize_st
   measure turbine rotation
   measure windspeed
   measure power output
   optimize pitch angle (and load?)
*/

enum states_t // these are the different states for operations
{
  Default_st,
  Startup_st,
  Optimize_st,
  Survival_st,
} currentState;

void setup() {
  // Sets which pins are input and which are output:
  pinMode(ESTOP_SWITCH_PIN, INPUT);

  Serial.begin(9600);

  brakeservo.attach(10);  // attaches the servo on pin 10 to the servo object
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

}

void loop() {
  if (Serial.available() > 0) {
    pitch_angle = Serial.read();
  }
  encoder();

  int estopReading = digitalRead(ESTOP_SWITCH_PIN);

  switch (currentState)
  {
    // This is the state when load is disconnected or if the estop is pressed
    case Default_st:
      //Check the estop button, if the button has been released send the turbine to start up state
      if (estopReading == HIGH) {
        currentState = Startup_st; // Switches the state
      }
      currentPitchAngle = 0; // Pitch the turbine out of the wind to slow the blades rotation
      //writePitch(currentPitchAngle); // Calls the function to change the pitch angle
      pitch_angle = 50;
      set_pitch();
      delay(50); // Give time for the linear actuator to move
      //Check whether the mechanical break has been applied
      if (angle != 0) {
        applybrake(); //Call the function to apply the brake
      }
      angle = 0; //sets the angle of the break servo motor to zero
      break;

    case Startup_st:    // This state gives the lowest cut-in speed and easiest restart
      if (estopReading == LOW) {
        currentState = Default_st;
      }
      pitch_angle = 60;
      set_pitch();
      if (angle != max_angle) {
        releasebrake(); //releases the mecanical brake
      }
      angle = max_angle;//makes sure the brake stays released
      //measure_rpm();
      if (rps > 4) {
        currentState = Optimize_st;
      }
      break;

    // This state matches pitch and load to wind speed to optimize power
    case Optimize_st:
      if (estopReading == LOW) {
        currentState = Default_st;
      }
      pitch_angle = 90;
      //Serial.println("Go to 90");
      set_pitch();
      if (rps < 4) {
        currentState = Startup_st;
      }
      break;

    case Survival_st:
      if (estopReading == LOW) {
        currentState = Default_st;
      }
  }

}

void releasebrake() {
  for (angle = max_angle; angle >= 0; angle -= 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    brakeservo.write(angle);              // tell servo to go to position in variable 'angle'
    delay(15);                            // waits 15ms for the servo to reach the position
  }
}

void applybrake() {
  for (angle = 0; angle <= max_angle; angle += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    brakeservo.write(angle);                       // tell servo to go to position in variable 'angle'
    delay(15);
  }
}

void encoder() {
  long newPosition = myEnc.read();

  if (newPosition != oldPosition) {
    long newtime = millis();
    dx = (newPosition - oldPosition);
    dt = newtime - oldtime;
    oldPosition = newPosition;
    oldtime = newtime;
    cps = dx / (dt / 1000);
    rps = cps / 2048;
    //Serial.println(newPosition/2048);
    Serial.println(rps);
  }
}

void set_pitch() {
  if (pitch_angle < current_angle) {
    //Serial.println("contracting to:");
    for (pos = current_angle; pos >= pitch_angle; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    current_angle = pitch_angle;
  }
  if (pitch_angle > current_angle) {
    //Serial.println("extending to:");
    for (pos = current_angle; pos <= pitch_angle; pos += 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    current_angle = pitch_angle;
  }
  delay(15);
}
