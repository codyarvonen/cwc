/**
   WindControl.ino

   This contains all the code required for the competition
   For a wiring diagram, see WD004

   Written for the Collegiate Wind Competition
   by the 2021 (quarantine) team 49

   Contributors: Jacob Roberts James Cutler Sarah Cheng
*/
#include <Servo.h>

/* PINS */
#define VOLTAGE_INPUT_PIN A0
#define ESTOP_SWITCH_PIN 2
#define PITCH_CONTROL_PIN 9
#define BACK_PITCH_CONTROL_PIN 10
#define RPM_INPUT_PIN A1
#define FRONT_MOTOR_CONTROL_PIN 3
#define BACK_MOTOR_CONTROL_PIN 4
#define RELAY_PIN_1 5
#define RELAY_PIN_2 6
#define RELAY_PIN_3 7
#define RELAY_PIN_4 8
#define RELAY_PIN_5 11
#define RELAY_PIN_6 12

#define RATED_POWER_ENTER_THRESHOLD 11.90
#define RATED_POWER_EXIT_THRESHOLD 5.0
#define NUMBER_OF_PITCH_ANGLES 9
#define NUMBER_OF_PWM_VALUES 15
#define NUMBER_OF_WIND_SPEEDS 5
#define MAX_PITCH_ANGLE 180
#define MIN_PITCH_ANGLE 0
#define MAX_PWM_VALUE 15
#define MAX_WIND_SPEED 11
#define MIN_WIND_SPEED 6

#define PITCH_TABLE_NAME speedPitchVoltageTable
#define PWM_TABLE_NAME speedPWMVoltageTable

#define MIN_PITCH_MICROSECONDS 1480
#define MAX_PITCH_MICROSECONDS 1820
#define MAX_PITCH_MICROSECONDS_BACK 2000
#define MIN_PITCH_MICROSECONDS_BACK 1550

/* Rated Power Constants */
#define RP_DESIRED_VOLTAGE 11.8

/* PID Constants */
#define TS 0.1
#define KP 30
#define KD 3
#define KI -0.5
#define FORCE_MAX 30.0
#define KM 0.05
#define SIGMA 0.05

/* Optimization Constants */
#define CONVERGENCE_TOLERANCE 0.2

/* Debug Values */
#define PRINT_RPM 1
#define PRINT_VOLTAGE 0
#define PRINT_AVERAGE_VOLTAGE 1
#define PRINT_WIND_SPEED 0
#define PRINT_PITCH_INDEX 0
#define PRINT_WIND_SPEED_INDEX 0
#define PRINT_PID_OUTPUT 1
#define PRINT_RESISTANCE 0
#define PRINT_POWER 0

// Holds each of the states possible
enum states_t
{
  startup_st,
  optimize_st,
  estop_arming_st,
  estop_armed_st,
  rated_power_st
} currentState;

// Fully extended = pitchControl.write(180) = fully perpendicular to wind

static const float speedPitchVoltageTable[NUMBER_OF_PITCH_ANGLES][NUMBER_OF_WIND_SPEEDS] = {
  {0.0, 0.0, 0.0, 0.0, 0.0},
  {0.00, 0.00, 0.00, 0.05, 0.00},
  {0.00, 0.05, 0.09, 0.18, 0.00},
  {0.05, 0.18, 0.27, 0.32, 0.00},
  {0.18, 0.27, 0.50, 0.55, 0.00},
  {0.32, 0.55, 0.77, 0.96, 0.00},
  {0.68, 0.87, 1.18, 1.55, 0.36},
  {0.91, 1.23, 1.50, 1.68, 1.59},
  {0.82, 1.14, 1.59, 1.59, 1.91}
}; // Represents the current voltage for a given pitch angle and wind speed

static const float speedPWMVoltageTable[NUMBER_OF_PWM_VALUES][NUMBER_OF_WIND_SPEEDS] = {
  {1.0, 2.0, 3.0},
  {1.0, 2.0, 3.0},
  {0.0, 5.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 5.0, 0.0}
};

float resistanceValues[64] = {
  99999, 5.877, 4.065, 3.04,
  2.402937538, 2.03, 2.003597623, 1.739282196,
  1.508828886, 1.404, 1.353888433, 1.342093321,
  1.217199211, 1.13326576, 1.100390691, 1.043565551, 1.027,
  1.008356201, 0.960432043, 0.936714916, 0.88620427,
  0.874229287, 0.829970879, 0.825523249, 0.819865475, 0.807940108, 0.776879723,
  0.767661667, 0.727264058, 0.719493234, 0.689244473,
  0.686174431, 0.681979064, 0.678973264, 0.651971695,
  0.645719666, 0.616895972, 0.611069333, 0.59313369,
  0.586866942, 0.584001922, 0.581796347, 0.561857291,
  0.557019886, 0.538759609, 0.531214715, 0.517608255,
  0.512829397, 0.508796356,  0.496300596, 0.489890977,
  0.475710722, 0.459016403, 0.45765276, 0.452197043,
  0.442299709, 0.425762692, 0.412443615, 0.411342318,
  0.398800607, 0.385396749, 0.373458514, 0.363171344, 0.342035158,
};

uint8_t resistorMap[64] = {
  0, 1, 2, 4, 3, 8, 5, 6, 9, 16, 10, 7, 12, 17, 11,
  18, 32, 13, 20, 14, 19, 33, 24, 21, 34, 15, 22, 36,
  25, 35, 26, 23, 40, 37, 28, 38, 27, 41, 48, 29, 42,
  39, 30, 44, 49, 43, 50, 31, 45, 52, 46, 51, 56, 53,
  47, 54, 57, 58, 55, 60, 59, 61, 62, 63,
};

/* State variables */
float voltage_d1;
float voltage_d2;
float voltage_d3;
float currentPitchAngle;
float currentPWMValue;
float currentVoltage;
bool top = false;
unsigned long RPM = 0;
unsigned long PreviousRPM = 0;
unsigned long RPM_lastTriggeredTime = 0;
unsigned long RPM_hasntBeenTriggeredFor = 0;
bool RPM_notTriggered = false;
bool RPM_first = true;
unsigned int stateCounter = 0;
int currentLoadIndex = 1;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

enum states_t estopRetState;

/*PID control */
float beta = ((2 * SIGMA - TS) / (2 * SIGMA + TS));
float yDot = 0.0;
float previousY = 0.0;
float errorDot = 0.0;
float previousError = 0.0;
float integrator = 0.0;

/* Optimization variables */
float alpha = 1;
float gradient = 4;

Servo pitchControl;
Servo backPitchControl;

void setup()
{
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);
  pinMode(RELAY_PIN_5, OUTPUT);
  pinMode(RELAY_PIN_6, OUTPUT);
  pinMode(ESTOP_SWITCH_PIN, INPUT);
  pinMode(FRONT_MOTOR_CONTROL_PIN, OUTPUT);
  pinMode(BACK_MOTOR_CONTROL_PIN, OUTPUT);
  pinMode(PITCH_CONTROL_PIN, OUTPUT);
  pinMode(BACK_PITCH_CONTROL_PIN, OUTPUT);

  // Set the baud rate for printing to console
  Serial.begin(9600);
  Serial.println("\nWindControl");

  // Set the initial values
  currentPitchAngle = 100.0;
  currentPWMValue = 0.0;
  currentVoltage = 0.0;

  // Setup the state machine
  currentState = startup_st; // Always begin in the startup state
  turnOnBackMotor();

  // attaches the pitch system to pin 26
  pitchControl.attach(PITCH_CONTROL_PIN);
  backPitchControl.attach(BACK_PITCH_CONTROL_PIN);
  writePitch(currentPitchAngle);
  delay(200);
  normalBack();
  delay(200);

  changeResistance(2);
}

int i = 0;
bool forceEstop = false;

void loop()
{
  //  changeResistance(5);
  //  return;
  debugStatePrint();

  stateCounter++;

  /**************************/
  /** MEASURE FROM SENSORS **/
  /**************************/
  float voltage = measureVoltage();
  int estopReading = digitalRead(ESTOP_SWITCH_PIN);
  uint8_t windSpeedIndex = lookupWindSpeedIndex(voltage);
  // Use the wind speed index to find the actual wind speed
  float windSpeed = mapIndexToWindSpeed(windSpeedIndex);
  if (PRINT_WIND_SPEED && stateCounter % 200 == 0) {
    Serial.print("Wind speed is: ");
    Serial.println(windSpeed);
  }

  int valueQRD = analogRead(RPM_INPUT_PIN);

  if (valueQRD > 920) {
    RPM_notTriggered = true;
    RPM_hasntBeenTriggeredFor++;
  }
  if (valueQRD < 880 && RPM_notTriggered) {
    // Only do this if the sensor has been triggered
    unsigned long currentTime = millis();
    PreviousRPM = RPM;
    RPM_hasntBeenTriggeredFor = 0;
    RPM = 60 / ((currentTime - RPM_lastTriggeredTime) * 0.001);
    Serial.print("measured RPM is: ");
    Serial.println(RPM);
    RPM_lastTriggeredTime = currentTime;
    RPM_notTriggered = false;
    if (PRINT_RPM && stateCounter % 30 > 0) {
      Serial.print("special measured RPM is: ");
      Serial.println(RPM);
      float current_RPM = RPM;
    }
  }
  bool probablyStopped = false;

//100
  if (RPM_hasntBeenTriggeredFor > 50) {
    probablyStopped = true;
    RPM = 0;
  }

  if (PRINT_VOLTAGE && stateCounter % 200 == 0)
  {
    Serial.print("measured Voltage: ");
    Serial.println(voltage);
  }

  // Update voltage delayed measurements
  voltage_d3 = voltage_d2;
  voltage_d2 = voltage_d1;
  voltage_d1 = voltage;

  float averageVoltage = (voltage_d1 + voltage_d2 + voltage_d3) / 3;
  if (PRINT_AVERAGE_VOLTAGE && stateCounter % 30 == 0) {
    Serial.print("Average Voltage: ");
    Serial.println(averageVoltage);
  }

  /**************************/
  /* DECLARE VARIABLES USED */
  /**************************/
  float optimalPitchAngle;
  uint8_t optimalPitchIndex;
  float pidOutput;

  /**************************/
  /****** TRANSITIONS *******/
  /**************************/
  switch (currentState)
  {
    case startup_st:
      if (estopReading == LOW)
      {
        currentState = estop_arming_st;
        lastDebounceTime = millis(); // Record the last time the estop state changed
        estopRetState = startup_st;
        break;
      }
      else if (RPM > 50) {
        currentState = optimize_st;
        turnOnFrontMotor();
        stateCounter = 0;
        estopBack();
        delay(1000);
        // During the optimize state, we should keep the pitch angle at its best angle
        //writePitch(MAX_PITCH_ANGLE - 50);
        pitchControl.writeMicroseconds(MAX_PITCH_MICROSECONDS - 150);
        delay(1000);
      }
      break;
    // Optimize will transition to estop
    case optimize_st:
      estopRetState = optimize_st;
      if (estopReading == LOW)
      {
        currentState = estop_arming_st;
        lastDebounceTime = millis(); // Record the last time the estop state changed
        estopRetState = optimize_st;
      } else if (averageVoltage > RATED_POWER_ENTER_THRESHOLD) {
        Serial.println("Entering the Rated Power Loop");
        currentPitchAngle = 170;
        currentState = rated_power_st;
      }
      else if (!probablyStopped && averageVoltage < 0.05) {
        currentState = estop_armed_st;
        forceEstop = true;
        pitchControl.writeMicroseconds(MIN_PITCH_MICROSECONDS);
        delay(300);
        estopBack();
        delay(300);
        changeResistance(64);
        delay(300);
        estopRetState = startup_st;
        break;
      }
      else if (probablyStopped) {
        currentState = startup_st;
        turnOnBackMotor();
        stateCounter = 0;
        normalBack();
      }
      break;
    case estop_arming_st:
      // Check to see if the proper delay has passed.
      if ((millis() - lastDebounceTime) > debounceDelay)
      {
        if (estopReading == LOW)
        {
          // Transition to armed state
          currentState = estop_armed_st;
          pitchControl.writeMicroseconds(MIN_PITCH_MICROSECONDS);
          delay(300);
          estopBack();
          delay(300);
          changeResistance(64);
          delay(300);
        }
        else
        {
          // That was a debounce bug
          if (averageVoltage > RATED_POWER_ENTER_THRESHOLD)
          {
            currentState = rated_power_st;
          }
          else
          {
            currentState = optimize_st;
          }
        }
      }
      break;
    case estop_armed_st:
    {
      //delay(500);
      float previous_RPM = RPM;
      delay(500);
      float prev_voltage = measureVoltage();
      delay(500);
      float current_voltage = measureVoltage();
      //Does voltage difference become closer to 0 when reconnected to the load?
      Serial.print("voltage difference: ");
      Serial.println(abs(current_voltage-prev_voltage));
      float previous_diff = abs(current_voltage-prev_voltage);
      delay(500);
      prev_voltage = measureVoltage();
      delay(500);
      current_voltage = measureVoltage();
      float current_diff = abs(current_voltage-prev_voltage);
      Serial.print("double difference: ");
      Serial.println(abs(current_diff + previous_diff));
      if (forceEstop) {
        if (abs(current_diff + previous_diff) < 0.001)
        {
          Serial.print("reconnected to load: ");
          // transition to disarmed state
          currentState = startup_st;
          writePitch(MAX_PITCH_ANGLE);
          if (currentState == startup_st) {
            changeResistance(2);
            normalBack();
          } else if (currentState == optimize_st) {
            estopBack();
          }
          forceEstop = false;
        }
        break;
      }
      if (estopReading == HIGH)
      {
        Serial.print("entering estop disarming state: ");
        Serial.println(estopRetState);
     
        //lastDebounceTime = millis();
        currentState = estopRetState;
        writePitch(MAX_PITCH_ANGLE);
        if (currentState == startup_st) {
          Serial.print("entering startup state: ");
          changeResistance(2);
          normalBack();
        } else if (currentState == optimize_st) {
          estopBack();
        }
      }
    }
      break;
      
    case rated_power_st:
    {
      if (estopReading == LOW)
      {
        currentState = estop_arming_st;
        lastDebounceTime = millis(); // Record the last time the estop state changed
        estopRetState = optimize_st;
      } else if (averageVoltage < RATED_POWER_EXIT_THRESHOLD) {
        Serial.println("Leaving the Rated Power Loop");
        currentState = optimize_st;
      }
    }
      break;
  }

  /***************************/
  /******** ACTIONS **********/
  /***************************/
  switch (currentState)
  {
    case startup_st:
      if (stateCounter % 45 == 0) {
        if (top) {
          currentPitchAngle -= 3;
        } else {
          currentPitchAngle += 3;
        }

        if (currentPitchAngle >= 140) {
          top = true;
        } else if (currentPitchAngle <= 45) {
          top = false;
        }
        writePitch(currentPitchAngle);
      }
      break;
    case optimize_st:

      // During the optimize state, we should keep the pitch angle at its best angle
      //writePitch(MAX_PITCH_ANGLE - 50);
      pitchControl.writeMicroseconds(MAX_PITCH_MICROSECONDS - 90);
      delay(15);
      //      changeResistance(15);

      changeResistance(2);     
      //
      //      if (stateCounter % 30 == 0) {
      //      Serial.print("Power is ");
      //      Serial.println(averageVoltage * averageVoltage / 2);
      //
      //      }

      //      if (abs(RPM - PreviousRPM) > 5 || RPM_first) {
      //        RPM_first = false;
      //        gradient = 15;
      //        while (abs(gradient) > CONVERGENCE_TOLERANCE) {
      //          int search_dir = 1;
      //          // find search direction
      //          if (gradient > 0) {
      //            search_dir = 1;
      //          } else {
      //            search_dir = -1;
      //          }
      //          // new load resistance value
      //          int prev_LoadIndex = currentLoadIndex;
      //          currentLoadIndex = prev_LoadIndex + (alpha * search_dir);
      //          if (currentLoadIndex < 1) {
      //            currentLoadIndex = 1;
      //          } else if (currentLoadIndex > 15) {
      //            currentLoadIndex = 15;
      //          }
      //          uint8_t loadIndex = resistorMap[prev_LoadIndex];
      //          float current_load_res = resistanceValues[loadIndex];
      //
      //          float voltage_sum = 0;
      //          for (int i = 0; i < 5; i++) {
      //            voltage_sum += measureVoltage();
      //            delay(100);
      //          }
      //          float current_voltage = voltage_sum / 5;
      //          float previous_power = pow(current_voltage, 2) / (current_load_res);
      //          if (PRINT_POWER && stateCounter % 200 == 0) {
      //            Serial.print("\n At voltage: ");
      //            Serial.print(current_voltage);
      //            Serial.print(" and resistance: ");
      //            Serial.print(current_load_res);
      //            Serial.print("\nPrevious Power is ");
      //            Serial.println(previous_power);
      //          }
      //          // change load resistance
      //          changeResistance(currentLoadIndex);
      //          // include some kind of delay here
      //          delay(3000);
      //          loadIndex = resistorMap[currentLoadIndex];
      //          current_load_res = resistanceValues[loadIndex];
      //          voltage_sum = 0;
      //          for (int i = 0; i < 5; i++) {
      //            voltage_sum += measureVoltage();
      //            delay(100);
      //          }
      //          current_voltage = voltage_sum / 5;
      //          float current_power = pow(current_voltage, 2) / (current_load_res);
      //          if (PRINT_POWER) {
      //            Serial.print("\n At voltage: ");
      //            Serial.print(current_voltage);
      //            Serial.print(" and resistance: ");
      //            Serial.print(current_load_res);
      //            Serial.print("\nCurrent Power is ");
      //            Serial.println(current_power);
      //            Serial.print("\nGradient is ");
      //            Serial.println((current_power) - (previous_power));
      //          }
      //          gradient = (current_power) - (previous_power);
      //        }
      //      }
      break;
    case estop_arming_st: // Do Nothing
      break;
    case estop_armed_st: // Do Nothing (mealy action)
      break;
    case rated_power_st:
      //pitch control & call the PID functions
      pidOutput = pid(RP_DESIRED_VOLTAGE, averageVoltage);

      // Convert Output to Pitch Angle (output correlates positively with desired power direction)
      //      currentPitchAngle += pidOutput * KM;
      //      Serial.print("New pitch Angle is:");
      //      Serial.println(currentPitchAngle);

      int currentPitchMicroseconds = pitchControl.readMicroseconds();
      currentPitchMicroseconds += floor(pidOutput * KM);

      pitchControl.writeMicroseconds(currentPitchMicroseconds);
      Serial.print("New pitch is:");
      Serial.println(currentPitchMicroseconds);

      // Write Pitch to the Servo motor
      //writePitch(currentPitchAngle);
      delay(200);
      break;
  }
}

void changeResistance(int newIndex)
{
  int index = newIndex;
  if (newIndex > 63) {
    index = 63;
  } else if (newIndex < 1) {
    index = 1;
  }
  uint8_t bitset = resistorMap[index];

  if (PRINT_RESISTANCE && stateCounter % 200 == 0) {
    Serial.println();
    for (int i = 0; i < 6; i++) {
      Serial.print("bit number ");
      Serial.print(i);
      Serial.print(" is ");
      Serial.println(bitRead(bitset, i));
    }
  }
  digitalWrite(5, !bitRead(bitset, 0));
  digitalWrite(6, !bitRead(bitset, 1));
  digitalWrite(11, !bitRead(bitset, 2));
  digitalWrite(12, !bitRead(bitset, 3));
  digitalWrite(8, !bitRead(bitset, 4));
  digitalWrite(7, !bitRead(bitset, 5));
  currentLoadIndex = index;
}

void turnOnBackMotor() {
  Serial.println("Turning on the back motor");
  digitalWrite(BACK_MOTOR_CONTROL_PIN, LOW);
  digitalWrite(FRONT_MOTOR_CONTROL_PIN, HIGH);
}

void turnOnFrontMotor() {
  Serial.println("Turning on the front motor");
  digitalWrite(BACK_MOTOR_CONTROL_PIN, HIGH);
  digitalWrite(FRONT_MOTOR_CONTROL_PIN, LOW);
}
void turnOffBothMotors() {
  Serial.println("Turning off both motors");
  digitalWrite(BACK_MOTOR_CONTROL_PIN, HIGH);
  digitalWrite(FRONT_MOTOR_CONTROL_PIN, HIGH);
}

void writePitch(float pitchAngle)
{
  int writeValue = (int)(pitchAngle * 2.05 + 1450.5);
  if (writeValue > MAX_PITCH_MICROSECONDS) // Saturate MAX
    writeValue = MAX_PITCH_MICROSECONDS;
  else if (writeValue < MIN_PITCH_MICROSECONDS) // Saturate MIN
    writeValue = MIN_PITCH_MICROSECONDS;
  //  Serial.println("Writing pitch");
  //  Serial.println(writeValue);
  pitchControl.writeMicroseconds(writeValue);
}

void estopBack() {
  backPitchControl.writeMicroseconds(MIN_PITCH_MICROSECONDS_BACK);
}

void normalBack() {
  backPitchControl.writeMicroseconds(MAX_PITCH_MICROSECONDS_BACK);
}

float measureVoltage()
{
  int voltageMeasured = analogRead(VOLTAGE_INPUT_PIN);
  // Convert Voltage. see https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
  // 5V * 9.6 = 48V
  return voltageMeasured * 0.0049 * 9.292; // Will range between 0 and 45 (V). ADC constant, voltage divider constant.
}

uint8_t lookupWindSpeedIndex(float voltage)
{
  // Lookup appropriate pitch angle
  uint8_t currentPitchIndex = mapPitchAngleToIndex(currentPitchAngle);
  // uint8_t currentPWMIndex = mapPWMToIndex(currentPWMValue);
  if (PRINT_PITCH_INDEX && stateCounter % 200 == 0)
  {
    Serial.print("current Pitch Index: ");
    Serial.println(currentPitchIndex);
  }

  // loop through the table and find a pitch angle that is close
  uint8_t windSpeedIndex = findWindSpeedIndex(currentPitchIndex, voltage);
  // int8_t windSpeedIndex = findWindSpeedIndex(currentPWMIndex, voltage);
  if (PRINT_WIND_SPEED_INDEX && stateCounter % 200 == 0)
  {
    Serial.print("wind Speed Index: ");
    Serial.println(windSpeedIndex);
  }
  return windSpeedIndex;
}

// Take the pitch angle and voltage, and return the index of the wind speed
uint8_t findWindSpeedIndex(uint8_t pitchAngle, float voltage)
{
  // Store the best values
  uint8_t optimalWindSpeedIndex = 0;
  float currentSmallestDifference = 50;
  // Loop through each windSpeed index
  for (int windSpeedIndex = 0; windSpeedIndex < NUMBER_OF_WIND_SPEEDS; windSpeedIndex++)
  {
    // Find diff between the value in the table and our measured voltage
    float difference = abs(PITCH_TABLE_NAME[pitchAngle][windSpeedIndex] - voltage);
    // If it's smaller than the previous best value, update the best value
    if (difference < currentSmallestDifference)
    {
      optimalWindSpeedIndex = windSpeedIndex;
      currentSmallestDifference = difference;
    }
  }

  return optimalWindSpeedIndex;
}

uint8_t mapValueToIndex(float value, uint8_t max, uint8_t min, uint8_t tableSize)
{
  float sizeOfBin = (max - min) / tableSize;
  for (int i = 0; i < tableSize; i++)
  {
    float binMin = i * sizeOfBin;
    float binMax = (i + 1) * sizeOfBin;
    if (value >= binMin && value < binMax)
    {
      return i;
    }
  }
  return -1;
}

float mapPitchIndexToAngle(uint8_t pitchIndex)
{
  float sizeOfBin = (MAX_PITCH_ANGLE - MIN_PITCH_ANGLE) / NUMBER_OF_PITCH_ANGLES;
  return pitchIndex * sizeOfBin;
}

float mapIndexToWindSpeed(uint8_t speedIndex)
{
  float sizeOfBin = (MAX_WIND_SPEED - MIN_WIND_SPEED) / NUMBER_OF_WIND_SPEEDS;
  return speedIndex * sizeOfBin + (MIN_WIND_SPEED);
}

float mapIndexToPWM(uint8_t pwmIndex)
{
  float sizeOfBin = (MAX_PWM_VALUE - 0) / NUMBER_OF_PWM_VALUES;
  return pwmIndex * sizeOfBin;
}

// Take the pitch Angle (range 0-15) and divide it evenly between 0 and WIND_SPEED_TABLE_SIZE_Y
uint8_t mapPWMToIndex(float PWMValue)
{
  return mapValueToIndex(PWMValue, MAX_PWM_VALUE, 0, NUMBER_OF_PWM_VALUES);
}

// Take the pitch Angle (range 0-15) and divide it evenly between 0 and WIND_SPEED_TABLE_SIZE_Y
uint8_t mapPitchAngleToIndex(float pitchAngle)
{
  return mapValueToIndex(pitchAngle, MAX_PITCH_ANGLE, MIN_PITCH_ANGLE, NUMBER_OF_PITCH_ANGLES);
}

uint8_t mapWindSpeedToIndex(float windSpeed)
{
  return mapValueToIndex(windSpeed, MAX_WIND_SPEED, MIN_WIND_SPEED, NUMBER_OF_WIND_SPEEDS);
}

// Take the windSpeed and return the index of the best pitch angle
uint8_t findOptimalPitchAngle(uint8_t windSpeedIndex)
{
  // Store the best values
  uint8_t optimalPitchAngleIndex = 0;
  float currentMaxVoltage = 0.0;
  // Loop through each pitch angle at the current windSpeed and find best value
  for (int pitchAngleIndex = 0; pitchAngleIndex < NUMBER_OF_PITCH_ANGLES; pitchAngleIndex++)
  {
    // Find the value in the table
    float voltage = PITCH_TABLE_NAME[pitchAngleIndex][windSpeedIndex];
    // If it's greater than the previous best value, update the best value
    if (voltage > currentMaxVoltage)
    {
      optimalPitchAngleIndex = pitchAngleIndex;
      currentMaxVoltage = voltage;
    }
  }
  return optimalPitchAngleIndex;
}

// Take the windSpeed and return the index of the best pitch angle
uint8_t findOptimalPWM(uint8_t windSpeedIndex)
{
  // Store the best values
  uint8_t optimalPWMIndex = 0;
  float currentMaxVoltage = 0.0;
  // Loop through each pitch angle at the current windSpeed and find best value
  for (int PWMIndex = 0; PWMIndex < NUMBER_OF_PWM_VALUES; PWMIndex++)
  {
    // Find the value in the table
    float voltage = PWM_TABLE_NAME[PWMIndex][windSpeedIndex];
    // If it's greater than the previous best value, update the best value
    if (voltage > currentMaxVoltage)
    {
      optimalPWMIndex = PWMIndex;
      currentMaxVoltage = voltage;
    }
  }
  return optimalPWMIndex;
}

float pid(float y_r, float y)
{
  float error = y_r - y;

  // Integrate error with anti-windup
  if (abs(errorDot) < 0.05)
  {
    integrator += (TS / 2) * (error + previousError);
  }

  // Differentiate error
  errorDot = beta * errorDot + (1 - beta) * ((error - previousError) / TS);

  // differentiate y
  yDot = beta * yDot + (1 - beta) * ((y - previousY) / TS);
  // PID control
  float u_unsat = KP * error + KI * integrator - KD * yDot;

  float u_sat = saturate(u_unsat);

  // update delayed variables
  previousError = error;
  previousY = y;

  if (PRINT_PID_OUTPUT) {
    Serial.println("PID output is");
    Serial.println(u_sat);
  }
  return u_sat;
}

float saturate(float u)
{
  if (abs(u) > FORCE_MAX)
  {

    return FORCE_MAX * (u < 0 ? -1 : 1);
  }
  return u;
}

void debugStatePrint()
{
  static enum states_t previousState;
  static bool firstPass = true;
  // Only print the message if:
  // 1. This the first pass and the value for previousState is unknown.
  // 2. previousState != ticTacCurrentState - this prevents reprinting the same state name over and over.
  if (previousState != currentState || firstPass)
  {
    firstPass = false;            // previousState will be defined, firstPass is false.
    previousState = currentState; // keep track of the last state that you were in.
    switch (currentState)
    { // This prints messages based upon the state that you were in.
      case startup_st:
        Serial.println("-----------startup_st");
        break;
      case optimize_st:
        Serial.println("-----------optimize_st");
        break;
      case estop_arming_st:
        Serial.println("-----------estop_arming_st");
        break;
      case estop_armed_st:
        Serial.println("-----------estop_armed_st");
        break;
      case rated_power_st:
        Serial.println("-----------rated_power_st");
        break;
    }
  }
}
