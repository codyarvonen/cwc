// https://github.com/arduino-libraries/Servo
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

#define ENCODER_PIN1 2
#define ENCODER_PIN2 3
#define ACTUATOR_PIN 12
#define BRAKE_SERVO_PIN 9
#define ENGAGED_BRAKE_ANGLE 90
#define DISENGAGED_BRAKE_ANGLE 110
#define ACTUATOR_MAX 100
#define ACTUATOR_MIN 0
#define LOAD_DISCONNECT_THRESHOLD 0.1
#define INITIAL_PITCH 30 // This value is arbitrary, needs to be tested
#define BRAKE_PITCH 0 // This value is arbitrary, needs to be tested
#define PRIMARY_SWITCH_SIGNAL_PIN 10 // This value is arbitrary and needs to be set
#define SECONDARY_SWITCH_SIGNAL_PIN 11 // This value is arbitrary and needs to be set

bool brakeIsEngaged = false;

Servo pitchControl;
Servo brakeControl;

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

enum states_t {
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
    brakeControl.attach(BRAKE_SERVO_PIN);
    pinMode(PRIMARY_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(SECONDARY_SWITCH_SIGNAL_PIN, OUTPUT);
}

// unsigned int integerValue = 0;
// char incomingByte;
int pitch;
bool restartProtocolComplete = false;

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
        }
        else if (restartProtocolComplete) {
            restartProtocolComplete = false;
            currentState = power_curve;
        }
        break;
    case power_curve:
        if (toggle_test()) {
            currentState = testing;
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if(avgWindSpeed >= 11){
            currentState = steady_power;
        }
        if(engage_E_Stop()){
          currentState = emergency_shutdown;
        }
        break;
    case steady_power:
        if (toggle_test()) {
            currentState = testing;
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if(avgWindSpeed > 14){
            currentState = survival;
        }
        else if(avgWindSpeed < 11){
          currentState = steady_power;
        }
        if(engage_E_Stop()){
          currentState = emergency_shutdown;
        }
        break;
    case survival:
        if (toggle_test()) {
            currentState = testing;
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in initial position?
        if(avgWindSpeed < 14){
          currentState = steady_power;
        }
        if(engage_E_Stop()){
          currentState = emergency_shutdown;
        }
        break;
    case emergency_shutdown:
        if (toggle_test()) {
            currentState = testing;
        }
        // Set pitch to zero, trigger switches
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
        set_pitch(INITIAL_PITCH)
        if(currentPitch == INITIAL_PITCH){
          set_switches(true);
          restartProtocolComplete = true;
        }
        break;

    // Adjust the load to reach maximum Cp at each wind speed 5-11 m/s
    // TODO: test ideal resistor values for varying wind speeds and fixed pitch angles
    case power_curve:
      set_load(8); // set this load based on current power Cp? Current wind speed? current rpm?
      // TODO: Create function that uses lookup table(s) to determine and set resistor value
      break;

    // Adjust the pitch to maintain constant power output from 11-14 m/s
    // TODO: test pitch angles to have a fixed rpm for varying wind speeds and set resistor value
    case steady_power:
        set_load(STEADY_POWER_RESISTOR); // move this so it is set right before moving to steady_power?
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

    // After pitch is set to brake state, Flip switches so components draw from the wall and the generator runs
    // to the brake circuit. Hold this state until button is disengaged, or load reconnected.
    case emergency_shutdown:
    
      // Set switches to secondary state, opens load circuit, and powers the nacelle from the wall
      set_pitch(BRAKE_PITCH);

      // When we no longer need control, switch to run generator to brake circuit and prepare to draw power from wall for restart
      // check switch state so we only make the switch once, otherwise results in constant switch opening and closing
      if(currentPitch == BRAKE_PITCH && switchState == primary_switch_state){ 
        set_switches(false);
      }
      if(!engage_E_Stop()){
        currentState = restart;
      } 
      break;
    }

    /********************/
    /** MANUAL CONTROL **/
    /********************/
    // HANDLE COMMANDS FROM SERIAL MONITOR FOR PITCH AND BRAKE
    if (Serial.available() > 0) { // something came across serial
        // Do something with the value
        pitch = integerValue;
        if (pitchControl.attached()) {
            if (ACTUATOR_MIN <= pitch && pitch <= ACTUATOR_MAX) {
                Serial.print("Setting pitch to ");
                Serial.println(pitch);
                set_pitch(pitch);
            } else if (pitch == 19383) { // the decimal representation of 'BRAKE'
                if (!brakeIsEngaged) {
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

    // HANDLE ENCODER
    encoder();

    delay(1000);
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
}

void engageBrake() { brakeControl.write(ENGAGED_BRAKE_ANGLE); }

void disengageBrake() { brakeControl.write(DISENGAGED_BRAKE_ANGLE); }

void set_switches(bool initialState){
  /* The initial state has the load switch and generator to nacelle switch closed, and the wall to nacelle switch and brake switch open.
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

void set_load(int resistorValue){
  // TALK WITH POWER TO SEE WHAT SWITCHES THEY NEED AND HOW THE 
  // LOAD WILL BE IMPLEMENTED
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
    Serial.println(rpm);
    return rpm;
}

void engage_E_Stop() {
    int value;
    float volt;

    value = analogRead(VOL_PIN);

    volt = value * 5.0 / 1023.0;

    Serial.print("Value: ");
    Serial.print(value);
    Serial.print("  Volt: ");
    Serial.println(volt);
    if (value < LOAD_DISCONNECT_THRESHOLD) {
      Serial.println("NO LOAD DETECTED, BEGIN E-STOP");
      return true;
    }
    else if (estop_button_pressed){
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

test_type get_test_input() {}