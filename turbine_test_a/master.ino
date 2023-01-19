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

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
#define POWER_SWITCH_PIN 4
#define LOAD_NET_SWITCH_1_PIN 7
#define LOAD_NET_SWITCH_2_PIN 8
#define LOAD_NET_SWITCH_4_PIN 9
#define ACTUATOR_SWITCH_SIGNAL_PIN 10
#define EBRAKE_BTN_PIN 11
#define ACTUATOR_PIN 12
#define VOLTAGE_PIN 18
#define CURRENT_PIN 19

#define SERIAL_DATA_PIN 20
#define SERIAL_CLK_PIN 21

#define ACTUATOR_MAX 100
#define ACTUATOR_MIN 0
#define INITIAL_PITCH 30 // This value is arbitrary, needs to be tested
#define BRAKE_PITCH 0 // This value is arbitrary, needs to be tested

#define LOAD_DISCONNECT_THRESHOLD 0.05
#define STEADY_POWER_RPM 2500.0
#define STEADY_POWER_RANGE 100.0

// These are not ohm values, but rather which resistor configuration #1-8
const int lookupResistorTable[15]  =  {1, 2, 3, 4, 5, 6, 7, 8, 6,  6,  6,  6,  6,  6,  6}; 
// Pair of arrays acting as resistor lookup value given wind speed
const int lookupWindSpeedTable[15] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

float avgWindSpeed = 0.0;
const int sampleSize = 20;
float sampledWindSpeeds[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgRPM = 0.0;
float sampledRPM[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledVoltage[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledCurrent[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledPower[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgVoltage = 0.0;
float avgCurrent = 0.0;
float avgPower = 0.0; 

Servo pitchControl;

Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

typedef enum {
    restart,
    power_curve,
    steady_power,
    survival,
    emergency_shutdown,
    testing
} state;

state currentState;
state oldState;

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
    load_switch_test,
    toggle_test,
    wait
} test_type;

void setup() {
    Serial.begin(9600);

    pinMode(EBRAKE_BTN_PIN, INPUT_PULLUP);
    pinMode(POWER_SWITCH_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_1_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_2_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_4_PIN, OUTPUT);
    pinMode(ACTUATOR_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(ESTOP_LED, OUTPUT);
    pinMode(POWER_CURVE_LED, OUTPUT);
    pinMode(STEADY_POWER_LED, OUTPUT);
    pinMode(SURVIVAL_LED, OUTPUT);

    digital_write(POWER_SWITCH_PIN, LOW);
    digital_write(LOAD_NET_SWITCH_1_PIN, LOW);
    digital_write(LOAD_NET_SWITCH_2_PIN, LOW);
    digital_write(LOAD_NET_SWITCH_4_PIN, LOW);

    pitchControl.write(INITIAL_PITCH);
    pitchControl.attach(ACTUATOR_PIN);
    
    currentState = restart;

    // Setup LCD Screen
    //lcd.begin();
    //lcd.backlight();
    //lcd.print("Starting up the turbine");

    Serial.println("Starting up the turbine!\n");
    output_state_transition();
}


int pitch;
int resistorValue = 0;
bool restartProtocolComplete = false;
int currentPitch = 0;

bool found_ideal_resistor = false;
int tested_loads = 0;
int current_load = 0;
float oldRPM = 0;
float oldPower = 0.0;
int resistor_change_direction = 1;
const float RPM_VARIANCE = 1.1;
const float POWER_VARIANCE = 1.1;


void loop() {
    test_type test_state = get_input();
    oldState = currentState;
    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        if (test_state == toggle_test) {
            currentState = restart;
        } else {
          delay(1000);
        }
        break;
    case restart:
        if (test_state == toggle_test) {
            currentState = testing;
            
        } else if (is_load_connected()) {
            currentState = power_curve;
        }
        break;
    case power_curve:
        if (test_state == toggle_test) {
            currentState = testing;
            
        } else if (is_E_stop()) {
            currentState = emergency_shutdown;
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if (avgRPM >= 1100) {
            currentState = steady_power;
            
        }
        if(engage_E_Stop()){
            currentState = emergency_shutdown;
            set_state_led();
        }
        break;
    case steady_power:
        if (test_state == toggle_test) {
            currentState = testing;
            
        } else if (is_E_stop()) {
            currentState = emergency_shutdown;
            
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if (avgRPM > 2000) {
            currentState = survival;
            
        } else if (avgRPM < 1100) {
            currentState = power_curve;
            
        }
        
        break;
    case survival:
        if (test_state == toggle_test) {
            currentState = testing;
            
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in initial position?
        if (avgRPM < 2200) {
            currentState = steady_power;
            
        }
        if (!load_connected()) {
            currentState = emergency_shutdown;
            
        }
        break;
    case emergency_shutdown:
        if (test_state == toggle_test) {
            currentState = testing;
            
        }
        if(!engage_E_Stop()){
            currentState = restart;
            
        }
        break;
    }

    /********************/
    /** STATE ACTIONS ***/
    /********************/

    switch (currentState) {

    // This mode is designed for testing with manual control
    case testing:
        switch (test_state) {
        case pitch_test:
            test_pitch();
            break;
        case brake_test:
            if (brakeIsEngaged) {
                disengageBrake();
            } else {
                engageBrake();
            }
            break;
        case encoder_test:
            //Serial.println(encoder());
            break;
        case wind_speed_test:
            break;
        case switch_test:
            digital_write(GEN_LOAD_SWITCH_PIN, wallPower);
            // digital_write(LOAD_SWITCH_PIN, wallPower);
            digital_write(WALL_SWITCH_PIN, !wallPower);
            wallPower = !wallPower;
            break;
        case load_switch_test:
            test_load_switches();
            break;
        case wait:
            break;
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
    case power_curve:
        // Adjust resistor value until power output does not increase

        // Set new Resistor Value to test if ideal value not yet found
        if (!found_ideal_resistor){
            oldRPM = avgRPM;
            current_load += resistor_change_direction;
            set_load(current_load);

            // Check if new resistor improved power output
            // TODO: may need to account for small variability in power, grab n samples before comparing?
            if (avgPower > oldPower * POWER_VARIANCE){
                oldPower = avgPower;
            }
            else if (tested_loads >= 1){ 
                // If first change decreased power output, try moving in the other direction, otherwise we found our peak.
                found_ideal_resistor = true;
            }
            else {
                resistor_change_direction *= -1;
            }
            tested_loads++;
        } else if (avgRPM > oldRPM * RPM_VARIANCE){ //change 1.1 with constant var. Checks for change in RPM indicating new wind speed 
            found_ideal_resistor = false;
        }

        break;

    // Adjust the pitch to maintain constant power output from 11-14 m/s
    // TODO: test pitch angles to have a fixed rpm for varying wind speeds and set resistor value
    case steady_power:
        int curr_rpm = 0;
        currentPitch = 0;
    //set_load_resistance(6); // move this so it is set right before moving to steady_power?
        curr_rpm = encoder();   // Get a rolling average instead of single data point
        if (curr_rpm < STEADY_POWER_RPM - STEADY_POWER_RANGE) {
            currentPitch = pitchControl.read() + 5; // Find a good step size
            set_pitch(currentPitch);                // set this pitch based on current rpm
        } else if (curr_rpm > STEADY_POWER_RPM + STEADY_POWER_RANGE) {
            currentPitch = pitchControl.read() - 5; // Find a good step size
            set_pitch(currentPitch);                // set this pitch based on current rpm
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
    //sample_wind_speed();
    float rpm = encoder();
    update_averages();
    //Serial.println(rpm);    
    if (oldState != currentState){
      output_state_transition();
    }
    //write_to_screen();
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

void toggle_power_switch() {
    digitalWrite(POWER_SWITCH_PIN, externalPower);
    externalPower = !externalPower;
}
   
void set_load(int binary_val) {
    if (binary_val >= 8 || binary_val < 0) {
        Serial.println("Invalid load value");
    } else {
        if (binary_val & 0x1) {
            digital_write(LOAD_NET_SWITCH_1_PIN, HIGH);
        } else {
            digital_write(LOAD_NET_SWITCH_1_PIN, LOW);
        }
        if (binary_val & 0x2) {
            digital_write(LOAD_NET_SWITCH_2_PIN, HIGH);
        } else {
            digital_write(LOAD_NET_SWITCH_2_PIN, LOW);
        }
        if (binary_val & 0x4) {
            digital_write(LOAD_NET_SWITCH_4_PIN, HIGH);
        } else {
            digital_write(LOAD_NET_SWITCH_4_PIN, LOW);
        }
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

    update_avg_rpm(rpm);
    return rpm;
}


bool load_connected() {
    float voltage = analogRead(VOLTAGE_PIN) * 5.0 / 1023.0;
    if (voltage < LOAD_DISCONNECT_THRESHOLD) {
        return false;
    }
    return true;
}

void clear_buf() {
    // Eat up any extra bytes that may have gotten included
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void test_pitch() {
    Serial.println("Enter pitch value to test: ");
    int pitch = get_input_integer();
    if (pitch < ACTUATOR_MIN) {
        pitch = ACTUATOR_MIN;
    }
    if (pitch > ACTUATOR_MAX) {
        pitch = ACTUATOR_MAX;
    }
    set_pitch(pitch);
}

void test_load_switches() {
    Serial.println("Enter load value to test: ");

    int load = get_input_integer();

    set_load(load);
}

int get_input_integer() {
    int input = 0;
    while (!Serial.available())
        ;
    if (Serial.available() > 0) {
        while (true) {
            incomingByte = Serial.read();
            if (incomingByte == '\n') {
                break;
            }
            if (incomingByte == -1) {
                continue;
            }
            input *= 10; // shift left 1 decimal place
            input = ((incomingByte - 48) + input);
        }
        clear_buf();
    }
    return input;
}

test_type get_input() {
    /*
    Input types:
        pitch
        brake
        rpm
        wind
        switch
        test
        null
    */
    if (Serial.available() > 0) {
        Serial.println("SERIAL AVAILABLE");
        char inputString[7];
        for (int i = 0; i < 6; i++) {
            inputString[i] = Serial.read();
            if (inputString[i] == '\n') {
                inputString[i + 1] = '\0';
            }
        }
        clear_buf();
        Serial.print("WE RECEIVED: ");
        Serial.println(inputString);
        if (!strcmp(inputString, "test\n")) {
            return toggle_test;
        }
        if (!strcmp(inputString, "pitch\n")) {
            return pitch_test;
        }
        if (!strcmp(inputString, "brake\n")) {
            return brake_test;
        }
        if (!strcmp(inputString, "rpm\n")) {
            return encoder_test;
        }
        if (!strcmp(inputString, "wind\n")) {
            return wind_speed_test;
        }
        if (!strcmp(inputString, "switch\n")) {
            return switch_test;
        }
    }

}

void update_avg_rpm(float measuredRPM) {
    avgRPM = ((avgRPM * sampleSize) - sampledRPM[0] + measuredRPM) / sampleSize;
    for (int i = 0; i < sampleSize - 1; i++) {
        sampledRPM[i] = sampledRPM[i + 1];
    }
    sampledRPM[sampleSize - 1] = measuredRPM;
}

// Recalculate the avgWindSpeed, update the sampleWindSpeed array pushing out the oldest data point
// and adding the newest data point
void update_avg_wind_speed(float measuredSpeed) {
    avgWindSpeed = ((avgWindSpeed * sampleSize) - sampledWindSpeeds[0] + measuredSpeed) / sampleSize;
    
    for (int i = 0; i < (sampleSize - 1); i++) {
        sampledWindSpeeds[i] = sampledWindSpeeds[i + 1];
    }
    
    sampledWindSpeeds[sampleSize - 1] = measuredSpeed;
}

void sample_wind_speed(){
    // Sample the pitot tube and calculate the wind speed, or use a lookup table using rpm and pitch angle
    float currentWindSpeed = 5.0;
    update_avg_wind_speed(currentWindSpeed);
}

int lookupResistor(float avgWindSpeed){
    int roundedWindSpeed = (int)(avgWindSpeed + 0.5);
    int index = 0;
    for (int i = 0; i < (sizeof(lookupWindSpeedTable)/sizeof(lookupWindSpeedTable[0])); i++){
        if(lookupWindSpeedTable[i] == roundedWindSpeed){
            index = i;
        }
    }
    return lookupResistorTable[index];
}

//TODO: Change this to return the string form of the state, then use for printing out curr and prev state
void output_state_transition(){
    if (currentState == power_curve){
        Serial.println("Entering Power Curve");    
    }
    else if (currentState == restart){
        Serial.println("Entering the Restart State");
    }
    else if (currentState == steady_power){
        Serial.println("Entering the Steady Power State");
    }
    else if (currentState == survival){
        Serial.println("Entering the Survival State");
    }
    else if (currentState == emergency_shutdown){
        Serial.println("Entering Emergency Shutdown");
    }
    else if (currentState == testing){
        Serial.println("Entering the Testing State");
    }
    else{
        Serial.println("Unrecognized State");
    }
    Serial.print("Current Avg RPM: ");
    Serial.println(avgRPM);
}

// Writes current status of turbine to external LCD screen
void write_to_screen(){

    //lcd.print("curr State:  prev State:  V: avgV, I: avgI, P: avgP, RPM: avgRPM, R: currLoad")
}

void update_averages(){
     float voltage = analogRead(VOLTAGE_PIN) * 5 / 1023; 

    // Current sensor has a sensitivity of 185 mV/A, and an offset of 2.5 V (0 A = 2.5 V)
    float current = ((analogRead(CURRENT_PIN) * 5 / 1023) - 2.5) / 0.185; 

    float power = voltage * current; // Watt

    avgVoltage = ((avgVoltage * sampleSize) - sampledVoltage[0] + voltage) / sampleSize;
    for (int i = 0; i< sampleSize - 1; i++) {
        sampledVoltage[i] = sampledVoltage[i + 1];
    }
    sampledVoltage[sampleSize - 1] = voltage;

    avgCurrent = ((avgCurrent * sampleSize) - sampledCurrent[0] + current) / sampleSize;
    for (int i = 0; i < sampleSize - 1; i++) {
        sampledCurrent[i] = sampledCurrent[i + 1];
    }
    sampledCurrent[sampleSize - 1] = current;

    avgPower = ((avgPower * sampleSize) - sampledPower[0] + power) / sampleSize;
    for (int i = 0; i < sampleSize - 1; i++) {
        sampledPower[i] = sampledPower[i + 1];
    }
    sampledPower[sampleSize - 1] = power;
}