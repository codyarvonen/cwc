// https://github.com/arduino-libraries/Servo

/************************************************************************************************************ 
TESTS TO RUN 
1 - Run tests with sampledRPM and sampledWindSpeeds to make sure the avgRPM and avgWindSpeed update correctly 
2 - Test ability to sense load disconnect and enter emergency shutdown
3 - Test ability to sense when load is reconnected, it will help to know how low of an RPM we can still sense
    a voltage across the load, and if we should change the resistor load to help with this.
4 - Test resistor relay, make sure that switches lead to different load resistances.
*************************************************************************************************************/
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

#define ENCODER_PIN1 2
#define ENCODER_PIN2 3
#define ACTUATOR_PIN 12
#define ACTUATOR_MAX 100
#define ACTUATOR_MIN 0
#define LOAD_DISCONNECT_THRESHOLD 0.05
#define INITIAL_PITCH 30 // This value is arbitrary, needs to be tested
#define BRAKE_PITCH 0 // This value is arbitrary, needs to be tested
#define ACTUATOR_SWITCH_SIGNAL_PIN 9
#define PRIMARY_SWITCH_SIGNAL_PIN 10 // This value is arbitrary and needs to be set
#define SECONDARY_SWITCH_SIGNAL_PIN 11 // This value is arbitrary and needs to be set
#define FIRST_RESISTOR_SWITCH 8
#define SECOND_RESISTOR_SWITCH 7
#define THIRD_RESISTOR_SWITCH 6

const int VOLTAGE_PIN = A0;
const int ESTOP_LED  = A1;
const int STEADY_POWER_LED = A2;
const int POWER_CURVE_LED = A3;
const int SURVIVAL_LED = A4;

const float STEADY_POWER_RPM = 2500.0;
const float STEADY_POWER_RANGE = 100.0;

float avgWindSpeed = 0.0;
float sampledWindSpeeds[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgRPM = 0.0;
float sampledRPM[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Servo pitchControl;

//   Change the two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

enum states_t {
    restart,
    power_curve,
    steady_power,
    survival,
    emergency_shutdown,
    testing
} currentState;

enum states_s {
  primary_switch_state,
  secondary_switch_state
} switchState;

typedef enum {
    pitch_test,
    brake_test,
    encoder_test,
    wind_speed_test,
    switch_test,
    exit_test
} test_type;

void setup() {
    Serial.begin(9600);
    Serial.write("Beginning test! \n");
    pitchControl.attach(ACTUATOR_PIN);
    pinMode(PRIMARY_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(SECONDARY_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(ACTUATOR_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(FIRST_RESISTOR_SWITCH, OUTPUT);
    pinMode(SECOND_RESISTOR_SWITCH, OUTPUT);
    pinMode(THIRD_RESISTOR_SWITCH, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(ESTOP_LED, OUTPUT);
    pinMode(POWER_CURVE_LED, OUTPUT);
    pinMode(STEADY_POWER_LED, OUTPUT);
    pinMode(SURVIVAL_LED, OUTPUT);

    currentState = restart;
    set_state_led();
}

// unsigned int integerValue = 0;
// char incomingByte;
int pitch;
int resistorValue = 0;
bool restartProtocolComplete = false;
int currentPitch = 0;


void loop() {

    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        // if (toggle_test()) {
        //     currentState = restart;
        // }
        break;
    case restart:
        if (toggle_test()) {
            currentState = testing;
            set_state_led();
        }
        else if (restartProtocolComplete) {
            restartProtocolComplete = false;
            currentState = power_curve;
            set_state_led();
        }
        break;
    case power_curve:
        if (toggle_test()) {
            currentState = testing;
            set_state_led();
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if(avgWindSpeed >= 11){
            currentState = steady_power;
            set_state_led();
        }
        if(engage_E_Stop()){
            currentState = emergency_shutdown;
            set_state_led();
        }
        break;
    case steady_power:
        if (toggle_test()) {
            currentState = testing;
            set_state_led();
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if(avgWindSpeed > 14){
            currentState = survival;
            set_state_led();
        }
        else if(avgWindSpeed < 11){
          currentState = steady_power;
          set_state_led();
        }
        if(engage_E_Stop()){
          currentState = emergency_shutdown;
          set_state_led();
        }
        break;
    case survival:
        if (toggle_test()) {
            currentState = testing;
            set_state_led();
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in initial position?
        if(avgWindSpeed < 14){
          currentState = steady_power;
          set_state_led();
        }
        if(engage_E_Stop()){
          currentState = emergency_shutdown;
          set_state_led();
        }
        break;
    case emergency_shutdown:
        if (toggle_test()) {
            currentState = testing;
            set_state_led();
        }
        if(!engage_E_Stop()){
            currentState = restart;
            set_state_led();
        } 
        break;
    }

    /********************/
    /** STATE ACTIONS ***/
    /********************/

    switch (currentState) {

    // This mode is designed for testing with manual control
    case testing:
        if (Serial.available() > 0) {
        }
        break;

    // Pitch blades out to inital state, trigger switches back so generator powers
    // nacelle components and the load
    case restart:
        currentPitch = pitchControl.read();
        //set switches to secondary state in order to draw power from the wall
        set_switches(false);
        set_pitch(INITIAL_PITCH);
        if(currentPitch == INITIAL_PITCH){
          set_switches(true);
          restartProtocolComplete = true;
        }
        break;

    // Adjust the load to reach maximum Cp at each wind speed 5-11 m/s
    // TODO: test ideal resistor values for varying wind speeds and fixed pitch angles
    case power_curve:
        resistorValue = lookupResistor(avgWindSpeed);
        set_load_resistance(resistorValue); // set this load based on current power Cp? Current wind speed? current rpm?
        // TODO: Create function that uses lookup table(s) to determine and set resistor value
      break;

    // Adjust the pitch to maintain constant power output from 11-14 m/s
    // TODO: test pitch angles to have a fixed rpm for varying wind speeds and set resistor value
    case steady_power:
        int curr_rpm = 0;
        currentPitch = 0;
        set_load_resistance(6); // move this so it is set right before moving to steady_power? 
        curr_rpm = encoder(); // Get a rolling average instead of single data point
        if (curr_rpm < STEADY_POWER_RPM - STEADY_POWER_RANGE){
          currentPitch = pitchControl.read() + 5; // Find a good step size
          set_pitch(currentPitch); // set this pitch based on current rpm
        }
        else if (curr_rpm > STEADY_POWER_RPM + STEADY_POWER_RANGE){
          currentPitch = pitchControl.read() - 5; // Find a good step size
          set_pitch(currentPitch); // set this pitch based on current rpm
        }
        
        break;

    // Adjust pitch and load to minimize rpm and stress in the system
    case survival:
        break;

    // After pitch is set to brake state, pitch is set to a the braking angle, this state is help until load is connected and button disengaged
    case emergency_shutdown:
      set_pitch(BRAKE_PITCH);
      break;
    }

    // HANDLE ENCODER AND WIND SPEED
    sample_wind_speed();
    encoder();
}

void set_pitch(int pitch_angle) {
    digitalWrite(ACTUATOR_SWITCH_SIGNAL_PIN, HIGH);
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
    digitalWrite(ACTUATOR_SWITCH_SIGNAL_PIN, LOW);
}

void set_state_led(){
    digitalWrite(ESTOP_LED, LOW);
    digitalWrite(POWER_CURVE_LED, LOW);
    digitalWrite(STEADY_POWER_LED, LOW);
    digitalWrite(SURVIVAL_LED, LOW);
    switch (currentState)
    {
        case testing:
            digitalWrite(ESTOP_LED, HIGH);
            digitalWrite(POWER_CURVE_LED, HIGH);
            digitalWrite(STEADY_POWER_LED, HIGH);
            digitalWrite(SURVIVAL_LED, HIGH);
            break;
        case restart:
            digitalWrite(ESTOP_LED, HIGH);
            digitalWrite(POWER_CURVE_LED, HIGH);
            break;
        case power_curve:
            digitalWrite(POWER_CURVE_LED, HIGH);
            break;
        case steady_power:
            digitalWrite(STEADY_POWER_LED, HIGH);
            break;
        case survival:
            digitalWrite(SURVIVAL_LED, HIGH);
            break;
        case emergency_shutdown:
            digitalWrite(ESTOP_LED, HIGH);
            break;
    }
    
}

void set_switches(bool initialState){
  /* The initial state has the load switch and generator to nacelle switch closed, and the wall to nacelle switch open.
  The generator to nacelle switch and the load switch share a digital signal. The wall to nacelle switch and the brake switch
  also share a digital signal. */

  //First set all switches to open, to avoid ever having all switches closed
  digitalWrite(PRIMARY_SWITCH_SIGNAL_PIN, LOW);
  digitalWrite(SECONDARY_SWITCH_SIGNAL_PIN, LOW);
  if(initialState){
    digitalWrite(PRIMARY_SWITCH_SIGNAL_PIN, HIGH);
  }
  else{
    digitalWrite(SECONDARY_SWITCH_SIGNAL_PIN, HIGH);
  }
}

float encoder() {
    static long oldPosition = 0;
    static long oldtime = 0;
    static float rpm = 0;

    long newPosition = myEnc.read();

    if (newPosition != oldPosition) {
        long newtime = micros();
        float dx = (newPosition - oldPosition);
        float dt = newtime - oldtime;
        oldPosition = newPosition;
        oldtime = newtime;
        rpm = (dx * 1000000 * 60) / (dt * 2048);
    }
    //Serial.println(rpm);
    update_avg_rpm(rpm);
    return rpm;
}

bool engage_E_Stop() {
    int value;
    float volt;

    value = analogRead(VOLTAGE_PIN);

    volt = value * 5.0 / 1023.0;

    Serial.print("Value: ");
    Serial.print(value);
    Serial.print("  Volt: ");
    Serial.println(volt);
    if (volt < LOAD_DISCONNECT_THRESHOLD) {
      Serial.println("NO LOAD DETECTED, BEGIN E-STOP");
      return true;
    }
    else if (false){ //check for ebutton press here
      Serial.println("ESTOP BUTTON ENGAGED, BEGIN E-STOP");
      return true;
    }
    else {
      return false;
    }
}

bool toggle_test() {
    if (Serial.available() > 0) {
        char inputString[6];
        for (int i = 0; i < 5; i++) {
            inputString[i] = Serial.read();
        }
        inputString[5] = '\0';
        // Eat up any extra bytes that may have gotten included
        while (Serial.available() > 0) {
            Serial.read();
        }
        return !strcmp(inputString, "test\n");
    }
    return false;
}

void update_avg_rpm(float measuredRPM){
    avgRPM = ((avgRPM*sizeof(sampledRPM)) - sampledRPM[0] + measuredRPM)/sizeof(sampledRPM);
    for(int i = 0; i < sizeof(sampledWindSpeeds) - 1; i++){
        sampledRPM[i] = sampledRPM[i+1];
    }
    sampledRPM[sizeof(sampledRPM)-1] = measuredRPM;
}

// Recalculate the avgWindSpeed, update the sampleWindSpeed array pushing out the oldest data point and adding the newest data point
void update_avg_wind_speed(float measuredSpeed){
    avgWindSpeed = ((avgWindSpeed*sizeof(sampledWindSpeeds)) - sampledWindSpeeds[0] + measuredSpeed)/sizeof(sampledWindSpeeds);
    for (int i = 0; i < (sizeof(sampledWindSpeeds) - 1); i++) {
        sampledWindSpeeds[i] = sampledWindSpeeds[i+1];
    }
    sampledWindSpeeds[sizeof(sampledWindSpeeds)-1] = measuredSpeed;
}

void sample_wind_speed(){
    // Sample the pitot tube and calculate the wind speed, or use a lookup table using rpm and pitch angle
    float currentWindSpeed = 5.0;
    update_avg_wind_speed(currentWindSpeed);
}

void set_load_resistance(int resistance){
    // Set the switches in the resistor relay to get the desired resistance
    if (resistance == 1){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, LOW);
        digitalWrite(SECOND_RESISTOR_SWITCH, LOW);
        digitalWrite(THIRD_RESISTOR_SWITCH, LOW);
    }
    else if (resistance == 2){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, HIGH);
        digitalWrite(SECOND_RESISTOR_SWITCH, LOW);
        digitalWrite(THIRD_RESISTOR_SWITCH, LOW);
    }
    else if (resistance == 3){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, LOW);
        digitalWrite(SECOND_RESISTOR_SWITCH, HIGH);
        digitalWrite(THIRD_RESISTOR_SWITCH, LOW);
    }
    else if (resistance == 4){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, HIGH);
        digitalWrite(SECOND_RESISTOR_SWITCH, HIGH);
        digitalWrite(THIRD_RESISTOR_SWITCH, LOW);
    }
    else if (resistance == 5){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, LOW);
        digitalWrite(SECOND_RESISTOR_SWITCH, LOW);
        digitalWrite(THIRD_RESISTOR_SWITCH, HIGH);
    }
    else if (resistance == 6){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, HIGH);
        digitalWrite(SECOND_RESISTOR_SWITCH, LOW);
        digitalWrite(THIRD_RESISTOR_SWITCH, HIGH);
    }
    else if (resistance == 7){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, LOW);
        digitalWrite(SECOND_RESISTOR_SWITCH, HIGH);
        digitalWrite(THIRD_RESISTOR_SWITCH, HIGH);
    }
    else if (resistance == 8){ //a load resistance of X ohms
        digitalWrite(FIRST_RESISTOR_SWITCH, HIGH);
        digitalWrite(SECOND_RESISTOR_SWITCH, HIGH);
        digitalWrite(THIRD_RESISTOR_SWITCH, HIGH);
    }
}

// Pair of arrays acting as resistor lookup value given wind speed
const int lookupWindSpeedTable[15] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
const int lookupResistorTable[15]  =  {1, 2, 3, 4, 5, 6, 7, 8, 6,  6,  6,  6,  6,  6,  6}; // These are not ohm values, but rather which resistor configuration #1-8

int lookupResistor(float avgWindSpeed){
    int roundedWindSpeed = (int)(avgWindSpeed + 0.5);
    int index = 0;
    for (int i = 0; i < sizeof(lookupWindSpeedTable); i++){
        if(lookupWindSpeedTable[i] == roundedWindSpeed){
            index = i;
        }
    }
    return lookupResistorTable[index];
}

test_type get_test_input() {}