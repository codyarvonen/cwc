// https://github.com/arduino-libraries/Servo

/************************************************************************************************************
TESTS TO RUN
1 - Test the new pitch control that takes in a degree input
2 - Test the calibrate_pitch test state
3 - Run state transition test
*************************************************************************************************************/
#include <LiquidCrystal_I2C.h> // Uses a zip library downloaded from https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
#define NACELLE_SWITCH_PIN 4
#define LOAD_SWITCH_PIN 12
#define LOAD_NET_SWITCH_1_PIN 46
#define LOAD_NET_SWITCH_2_PIN 47
#define LOAD_NET_SWITCH_4_PIN 48
#define LOAD_NET_SWITCH_8_PIN 49
#define ACTUATOR_SWITCH_SIGNAL_PIN 10
#define EBRAKE_BTN_PIN 18
//#define EBRAKE_BTN_PIN 13
#define ACTUATOR_PIN 5
#define VOLTAGE_PIN A12

#define ACTUATOR_MAX 130
#define ACTUATOR_MIN 60
#define INITIAL_PITCH 63
#define BRAKE_PITCH 115    // This value is arbitrary, needs to be tested

#define BASE_RESISTOR 1.0

#define LOAD_DISCONNECT_THRESHOLD 0.05
#define STEADY_POWER_RPM 2500.0
#define STEADY_POWER_RANGE 100.0

#define POWER_CURVE_RPM 900.0
#define POWER_CURVE_VOLTAGE 5.0

LiquidCrystal_I2C lcd(0x3F, 20, 4);  // 0x3F is the current lcd display address, this may need to be changed for other displays

// Pair of arrays acting as resistor lookup value given wind speed
//const int lookupWindSpeedTable[15] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

float avgWindSpeed = 0.0;
const int sampleSize = 20;
float sampledWindSpeeds[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgRPM = 0.0;
float prevRPM = 0.0;
int num_wind_tested = 0;
float pwr_at_11 = 0.0;
float voltage_factor 4.969 // UPDATE THIS VALUE TO MATCH INITIAL STATE
float sampledRPM[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledVoltage[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledPower[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgVoltage = 0.0;
float avgPower = 0.0; 

const float PWR_CURVE_THRESHOLD = 150.0;

bool externalPower = true;

Servo pitchControl;

Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

float resistorLookupTable[15] = {0.249, 1.8, 2.049, 8.0, 8.249, 9.8, 10.049, 46.4, 46.649, 48.2, 48.449, 54.4, 54.649, 56.2, 56.449};
// float resistorLookupTable[15] = {56.449, 56.2, 54.649, 54.4, 48.449, 48.2, 46.649, 46.4, 10.049, 9.8, 8.249, 8.0, 2.049, 1.8, 0.249};
enum states_t {
    restart,
    power_curve,
    steady_power,
    survival,
    emergency_shutdown,
    testing
} currentState;

states_t oldState;

enum states_s { primary_switch_state, secondary_switch_state } switchState;

typedef enum {
    pitch_test,
    calibrate_test,
    brake_test,
    encoder_test,
    wind_speed_test,
    switch_test,
    load_switch_test,
    load_disconnect_test,
    toggle_test,
    wait
} test_type;

// void eBrakeISR() {
//   static unsigned long last_interrupt_time = 0;
//   unsigned long interrupt_time = millis();
//   if (interrupt_time - last_interrupt_time > 1000) {
//     Serial.println("Entering EBRAKE Interrupt Service Routine");
//     set_pitch(ACTUATOR_MIN);
//     currentState = restart;
//     while (is_E_stop());
//   }
//   last_interrupt_time = interrupt_time;
// }

void setup() {
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();

    pinMode(EBRAKE_BTN_PIN, INPUT_PULLUP);
    pinMode(LOAD_SWITCH_PIN, OUTPUT);
    pinMode(NACELLE_SWITCH_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_1_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_2_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_4_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_8_PIN, OUTPUT);
    pinMode(ACTUATOR_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);

    // attachInterrupt(digitalPinToInterrupt(EBRAKE_BTN_PIN), eBrakeISR, RISING);

    digitalWrite(LOAD_SWITCH_PIN, HIGH);
    digitalWrite(NACELLE_SWITCH_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_8_PIN, LOW);

    digitalWrite(ACTUATOR_SWITCH_SIGNAL_PIN, LOW);

    pitchControl.write(INITIAL_PITCH);
    pitchControl.attach(ACTUATOR_PIN);

    currentState = restart;
    oldState = restart;

    Serial.write("Starting up the turbine!\n");

    set_pitch(INITIAL_PITCH);
    delay(1000);
    // internal_power_switch();
}

int pitch;
int resistorValue = 0;
bool restartProtocolComplete = false;
int currentPitch = 0;
bool increase = true;

void loop() {

    test_type test_state = get_input();

//    int val = pitchControl.read();
//    Serial.println(val);
//    if (val == 80) {
//      increase = false;
//    }
//    if (val == 45) {
//      increase = true;
//    }
//    if (increase) {
//      Serial.println("Increasing");
////        pitchControl.write(val+1);
//      set_pitch(val+1);
//    } else {
//      Serial.println("Decreasing");
////        pitchControl.write(val-1);
//      set_pitch(val-1);
//    }
//    delay(500);

//    optimize_pitch();

    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        if (test_state == toggle_test) {
            toggle_power_switch();
//            set_pitch(INITIAL_PITCH);
            toggle_power_switch();
            currentState = restart;
        } else {
            delay(1000);
        }
        break;
    case restart:
        if (test_state == toggle_test) {
          Serial.println("Testing!");
            currentState = testing;
        } else if (avgRPM >= POWER_CURVE_RPM && avgVoltage >= POWER_CURVE_VOLTAGE) {
            set_load(14);
            currentState = power_curve;
        }
        break;
    case power_curve:
        if (test_state == toggle_test) {
          Serial.println("Testing!");
            currentState = testing;
        } else if (is_E_stop() || !is_load_connected()) {
            currentState = emergency_shutdown;
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if (num_wind_tested >= 7) {
            currentState = steady_power;
        }
        break;
    case steady_power:
        if (test_state == toggle_test) {
          Serial.println("Testing!");
            currentState = testing;
        } else if (is_E_stop() || !is_load_connected()) {
            currentState = emergency_shutdown;
        }
        // TODO: Install anemometer to determine windspeed of 12 m/s or more, this state may be
        // redundant anyway, and can be accomplished with steady_power
        // if (avgRPM > prevRPM + SURVIVAL_THRESHOLD) {
        //     currentState = survival;
        // } 
        break;
    case survival:
        if (test_state == toggle_test) {
          Serial.println("Testing!");
            currentState = testing;
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in initial position?
        if (is_E_stop() || !is_load_connected()) {
            currentState = emergency_shutdown;
        }
        break;
    case emergency_shutdown:
        if (test_state == toggle_test) {
            currentState = testing;
        }
        if (!is_E_stop() && is_load_connected()) {
            set_pitch(INITIAL_PITCH);
            delay(1000);
            internal_power_switch();
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
//        Serial.println("Testing!");
        switch (test_state) {
        case pitch_test:
            test_pitch();
            break;
        case calibrate_test:
            calibrate_pitch();
            break;
        case brake_test:
            Serial.println("Testing EBRAKE");
            while (is_E_stop());
            digitalWrite(LOAD_SWITCH_PIN, HIGH);
            digitalWrite(NACELLE_SWITCH_PIN, LOW);
            externalPower = true;
            delay(1000);
            set_pitch(60);
        case load_disconnect_test:
            Serial.println("Testing Load Disconnect");
            while (is_load_connected());
            digitalWrite(LOAD_SWITCH_PIN, HIGH);
            digitalWrite(NACELLE_SWITCH_PIN, LOW);
            externalPower = true;
            delay(1000);
            set_pitch(60);
            break;
        case encoder_test:
            Serial.println(encoder());
            break;
        case wind_speed_test:
            break;
        case switch_test:
            toggle_power_switch();
            break;
         case load_switch_test:
             test_load_switches();
             break;
        case wait:
            break;
        }
        break;

    case restart:
        break;
        
    case power_curve:
        // TODO: Determine when to call the optimize_load function (we need to detect when the wind speed increases by 1m/s
        if (avgRPM > prevRPM + PWR_CURVE_THRESHOLD) {
          optimize_load();
          prevRPM = avgRPM;
          pwr_at_11 = avgPower;
          num_wind_tested++;
          
        }
        break;

    case steady_power:
        optimize_pitch();
        break;

    // Adjust pitch and load to minimize rpm and stress in the system
    case survival:
        optimize_pitch();
        break;

    case emergency_shutdown:
        break;
    }

    // HANDLE ENCODER AND WIND SPEED
    encoder();
    update_averages();
    write_to_lcd();
    
}

void optimize_pitch() {
  const float TARGET_MAX_PWR = pwr_at_11 + pwr_at_11 * 0.05;
  const float TARGET_MIN_PWR = pwr_at_11 - pwr_at_11 * 0.05;

  const float TARGET_MAX_RPM = prevRPM - prevRPM * 0.05;

  Serial.println(avgVoltage);
  if (avgPower < TARGET_MIN_PWR) {
    Serial.println("Power below the min");
    int current_angle = pitchControl.read();
    set_pitch(current_angle+1);
  }
  if (avgPower > TARGET_MAX_PWR || avgRPM > TARGET_MAX_RPM) {
    Serial.println("Power or RPM above the max");
    int current_angle = pitchControl.read();
    set_pitch(current_angle-1);
  }
}

void optimize_load() {
    float powerVals[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int resistor = 0; resistor < 15; resistor++) {
      set_load(resistor);
      delay(1000);
      for (int i = 0; i < sampleSize; i++){
        update_averages();
      }
      powerVals[resistor] = avgPower;
    }
    int max_power_resistor = 0;
    float max_power_val = 0.0;
    for (int i = 0; i < 15; i++) {
      if (powerVals[i] > max_power_val) {
        max_power_resistor = i;
      }
    } 
    set_load(max_power_resistor);
     
}

void set_pitch(int pitch_angle) {
  
    digitalWrite(ACTUATOR_SWITCH_SIGNAL_PIN, HIGH);
    delay(1000);
    int current_angle = pitchControl.read();
    Serial.println(current_angle);
    if (pitch_angle < current_angle) {
        for (int pos = current_angle-1; pos >= pitch_angle; pos--) {
            pitchControl.write(pos);
            delay(50);
        }
    }
    if (pitch_angle > current_angle) {
        for (int pos = current_angle+1; pos <= pitch_angle; pos++) {
            pitchControl.write(pos);
            delay(50);
        }
    }
    Serial.println(pitchControl.read());
    delay(1000);
    digitalWrite(ACTUATOR_SWITCH_SIGNAL_PIN, LOW);
}

void toggle_power_switch() {
    digitalWrite(LOAD_SWITCH_PIN, !externalPower);
    digitalWrite(NACELLE_SWITCH_PIN, externalPower);
    externalPower = !externalPower;
}

void external_power_switch() {
    digitalWrite(LOAD_SWITCH_PIN, HIGH);
    digitalWrite(NACELLE_SWITCH_PIN, LOW);
    externalPower = true;
}

void internal_power_switch() {
    digitalWrite(LOAD_SWITCH_PIN, LOW);
    digitalWrite(NACELLE_SWITCH_PIN, HIGH);
    externalPower = false;
}

int currentResistor = 15;
void set_load(int binary_val) {
    if (binary_val > 15 || binary_val <= 0) {
        Serial.println("Invalid load value");
    } else {
        currentResistor = binary_val;
        if (binary_val & 0x1) {
            digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
        } else {
            digitalWrite(LOAD_NET_SWITCH_1_PIN, HIGH);
        }
        if (binary_val & 0x2) {
            digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
        } else {
            digitalWrite(LOAD_NET_SWITCH_2_PIN, HIGH);
        }
        if (binary_val & 0x4) {
            digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);
        } else {
            digitalWrite(LOAD_NET_SWITCH_4_PIN, HIGH);
        }
        if (binary_val & 0x8) {
            digitalWrite(LOAD_NET_SWITCH_8_PIN, LOW);
        } else {
            digitalWrite(LOAD_NET_SWITCH_8_PIN, HIGH);
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

bool is_load_connected() {
    float voltage = (analogRead(VOLTAGE_PIN) / 1024.0) * 5.0 * VOLTAGE_FACTOR;
    return voltage != 0.0;
}

bool is_E_stop() {
     return !digitalRead(EBRAKE_BTN_PIN);
}

void clear_buf() {
    // Eat up any extra bytes that may have gotten included
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void test_pitch() {
    Serial.println("Enter pitch value to test in degrees: ");
    int actuatorPitch = get_input_integer();
//    int actuatorPitch = degrees_to_actuator(degrees);    
    if (actuatorPitch < ACTUATOR_MIN) {
        actuatorPitch = ACTUATOR_MIN;
    }
    if (actuatorPitch > ACTUATOR_MAX) {
        actuatorPitch = ACTUATOR_MAX;
    }
    set_pitch(actuatorPitch);
}

void calibrate_pitch() {
    Serial.println("Enter pitch value to test (actuator value): ");
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
            char incomingByte = Serial.read();
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
         calibrate
         rpm
         wind
         switch
         test
         null
     */
//     Serial.println("Checking for user input");
     if (Serial.available() > 0) {
         Serial.println("User input has been detected!");
         char inputString[8];
         int incomingByte;
         for (int i = 0; i < 8; i++) {
             incomingByte = Serial.read();
             if (incomingByte == '\n') {
                 inputString[i] = '\0';
                 break;
             } else {
                 inputString[i] = incomingByte;
             }
         }
         clear_buf();
         Serial.println(inputString);
         if (!strcmp(inputString, "test")) {
             return toggle_test;
         }
         if (!strcmp(inputString, "pitch")) {
             return pitch_test;
         }
         if (!strcmp(inputString, "brake")) {
             return brake_test;
         }
         if (!strcmp(inputString, "load")) {
             return load_disconnect_test;
         }
         if (!strcmp(inputString, "rpm")) {
             return encoder_test;
         }
         if (!strcmp(inputString, "wind")) {
             return wind_speed_test;
         }
         if (!strcmp(inputString, "switch")) {
             return switch_test;
         }
         if (!strcmp(inputString, "relay")) {
             return load_switch_test;
         }
         if (!strcmp(inputString, "relay")) {
             return calibrate_test;
         }
         
     }
     return wait;
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
    oldState = currentState;
    Serial.print("Current Avg RPM: ");
    Serial.println(avgRPM);
    Serial.print("Current Resistance: ");
     float resistance = resistorLookupTable[currentResistor] + BASE_RESISTOR;
    Serial.println(resistance);
    Serial.print("Current Voltage: ");
    Serial.println(avgVoltage);
    Serial.print("Current Power: ");
    Serial.println(avgPower);
}

void set_voltage_factor() {
    voltage_factor = (BASE_RESISTOR / (resistorLookupTable[currentResistor] + BASE_RESISTOR));
}
 
void update_averages() {
    float voltage = (analogRead(VOLTAGE_PIN) / 1024.0) * 5.0 * VOLTAGE_FACTOR; 
 
    float resistance = resistorLookupTable[currentResistor] + BASE_RESISTOR;

    float power = voltage * voltage / resistance;

    avgVoltage = ((avgVoltage * sampleSize) - sampledVoltage[0] + voltage) / sampleSize;
    for (int i = 0; i< sampleSize - 1; i++) {
        sampledVoltage[i] = sampledVoltage[i + 1];
    }
    sampledVoltage[sampleSize - 1] = voltage;

    avgPower = ((avgPower * sampleSize) - sampledPower[0] + power) / sampleSize;
    for (int i = 0; i < sampleSize - 1; i++) {
        sampledPower[i] = sampledPower[i + 1];
    }
    sampledPower[sampleSize - 1] = power;
}

void write_to_lcd(){
    // Print Voltage, Current, and Power on the first row
    lcd.setCursor (0, 0);            // go to the top left corner
    lcd.print("V:"); // write this string on the top row
    lcd.setCursor(2, 0);
    lcd.print(avgVoltage);
    lcd.setCursor(7, 0);
    lcd.print((char)0xF4);
    lcd.setCursor (8, 0);
    lcd.print(":");
    lcd.setCursor (9, 0);
    float resistance = resistorLookupTable[currentResistor] + BASE_RESISTOR;
    lcd.print(resistance);
    lcd.setCursor(15, 0);
    lcd.print("P:");
    lcd.setCursor(17, 0);
    lcd.print(avgPower);

    // Print RPM and pitch
    int pitch_in_degrees = actuator_to_degrees(currentPitch);
    lcd.setCursor (0, 1);            
    lcd.print("RPM:");
    lcd.setCursor (4, 1);
    lcd.print(avgRPM);
    lcd.setCursor (10, 1);
    lcd.print((char)0xF2);
    lcd.setCursor(11, 1);
    lcd.print(":");
    lcd.setCursor(12, 1);
    lcd.print(pitch_in_degrees);

    // Print the current state and pitch on the third row
    lcd.setCursor (0, 2);
    lcd.print("State:");
    lcd.setCursor (7, 2);
    if (currentState == restart){
      lcd.print("reset ");
    }
    else if (currentState == power_curve){
      lcd.print("curve ");
    }
    else if (currentState == steady_power){
      lcd.print("steady");
    }
    else if (currentState == survival){
      lcd.print("endure");
    }
    else if (currentState == emergency_shutdown){
      lcd.print("E Stop");
    }
    else if (currentState == testing){
      lcd.print(" test ");
    }
    else{
      lcd.print("error ");
    }
  
    // Print the power supply on the fourth row
    lcd.setCursor (0, 3);
    lcd.print("PWR Supply: ");
    lcd.setCursor (12, 3);
    if (externalPower){
      lcd.print("external");
    }
    else {
      lcd.print("internal");
    }
}

int actuator_to_degrees(int actuator_value){
  int degree = 90 * (actuator_value - ACTUATOR_MIN) / (ACTUATOR_MAX - ACTUATOR_MIN);
  return degree;
}

int degrees_to_actuator(int degrees){
  int actuator_value = ACTUATOR_MIN + (degrees / 90) * (ACTUATOR_MAX - ACTUATOR_MIN);
  return actuator_value;
}
