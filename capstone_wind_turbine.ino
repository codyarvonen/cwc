/**
   WindControl.ino

   This contains all the code required for the competition

   Written for the Collegiate Wind Competition
   by the 2022 team 47

   Contributors: Spencer Truman
   Revision: 2.0
   Last uploaded to box: 11 NOV 2021

   Revision History:
   Rev 1.0 - Initial code, basic arduino functionality and rough outline of states
   Rev 1.1 - Copy functions from last years code
   Rev 1.2 - Implement the ebrake functionality
   Rev 1.3 - Implement the linear acuator for pitch control
   Rev 1.4 - Implement the mechanical brake servo motor
   Rev 1.5 - Implement the rpm sensor
   Rev 2.0 - 
*/

#include <Servo.h>
Servo pitchControl; // This is the linear actuator to control pitch angle
Servo brakeservo; // This is the servo motor that controls the mechanical brake

/* PINS */
#define photoDiodePin A0 //reads the photo diode for the rpm sensor
#define ESTOP_SWITCH_PIN 2 //this is the same pin as the interrupt pin
#define PITCH_CONTROL_PIN 9 //controls the linear actuator for the pitch control
#define servoPin 10 //controls the servo motor for the mechanical brake

//define constants to be used throughout the code
#define MIN_PITCH_MICROSECONDS 1200 //the minimal value for the linear actuator
#define MAX_PITCH_MICROSECONDS 1820 //the maximum value for the linear actuator

//used in the rpm function
int counter;
int old;
int young;
float rpm;
int photoDiodeReading{0};
int photoDiodeVoltage{0};
unsigned long mytime;
unsigned long sensor_time;
unsigned long old_time;

//used in the mechanical brake functions
int angle{0};

//used in the pitch control function
float currentPitchAngle;

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
} currentState;

void setup() {
  // Sets which pins are input and which are output:
  pinMode(ESTOP_SWITCH_PIN, INPUT);
  pinMode(PITCH_CONTROL_PIN, OUTPUT);
  pinMode(ESTOP_SWITCH_PIN, INPUT_PULLUP);
  pinMode(photoDiodePin, INPUT); //pin A0
  attachInterrupt(digitalPinToInterrupt(ESTOP_SWITCH_PIN), estop, CHANGE);

  Serial.begin(9600);

  pitchControl.attach(PITCH_CONTROL_PIN);  // attaches the servo on pin 9 to the servo object
  writePitch(currentPitchAngle); //sets the initial blade pitch angle
  brakeservo.attach(10);  // attaches the servo on pin 10 to the servo object

}

void loop() {
  Serial.println(currentState);
  /**************************/
  /** MEASURE FROM SENSORS **/
  /**************************/
  //float voltage = measureVoltage();
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
      writePitch(currentPitchAngle); // Calls the function to change the pitch angle
      delay(50); // Give time for the linear actuator to move
      //Check whether the mechanical break has been applied
      if (angle != 0) {
        applybrake(); //Call the function to apply the brake
      }
      angle = 0; //sets the angle of the break servo motor to zero
      break;

    case Startup_st:    // This state gives the lowest cut-in speed and easiest restart
      if (estopReading == LOW) {
        currentState = Startup_st;
      }
      if (angle != 30) {
        releasebrake(); //releases the mecanical brake
      }
      angle = 30;//makes sure the brake stays released
      measure_rpm();
      if (rpm > 300) {
        currentState = Optimize_st;
      }
      currentPitchAngle = 90;
      writePitch(currentPitchAngle);
      break;

    // This state matches pitch and load to wind speed to optimize power
    case Optimize_st:
      if (estopReading == LOW) {
        currentState = Startup_st;
      }
      measure_rpm();
      if (rpm < 300) {
        currentState = Startup_st;
      }
      break;
      currentPitchAngle = 180;
      writePitch(currentPitchAngle);
  }
  //measure_rpm();

}

void writePitch(float pitchAngle)
{
  int writeValue = (int)(pitchAngle * 2.05 + 1200);
  if (writeValue > MAX_PITCH_MICROSECONDS) // Saturate MAX
    writeValue = MAX_PITCH_MICROSECONDS;
  else if (writeValue < MIN_PITCH_MICROSECONDS) // Saturate MIN
    writeValue = MIN_PITCH_MICROSECONDS;
  pitchControl.writeMicroseconds(writeValue);
}

void estop() {
  currentState = Default_st;
  Serial.println("Ebrake Pressed");
}

void measure_rpm() {
  Serial.println("Checking RPM");
  mytime = millis();
  while (millis() - 1000 < mytime) {
    photoDiodeReading = analogRead(photoDiodePin);
    photoDiodeVoltage = map(photoDiodeReading, 0, 1023, 0.0, 5.0);
    young = photoDiodeVoltage;

    if (young < old && young == 0) {
      counter++;
    }
    old = young;
  }
  rpm = counter * 60 / 2;
  counter = 0;
  Serial.print("rpm    ");
  Serial.println(rpm);
}

void applybrake() {
  for (angle = 30; angle >= 0; angle -= 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    //angle=0;
    brakeservo.write(angle);              // tell servo to go to position in variable 'angle'
    Serial.print("angle    ");
    Serial.println(angle);
    delay(15);                            // waits 15ms for the servo to reach the position
  }
}

void releasebrake() {
  for (angle = 0; angle <= 30; angle += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    brakeservo.write(angle);                       // tell servo to go to position in variable 'angle'
    Serial.print("angle    ");
    Serial.println(angle);
    delay(15);
  }
}
