// https://github.com/arduino-libraries/Servo

/************************************************************************************************************
ADDITIONS
- updated set_voltage_factor() (it was the inverse of what we need)
- updated state actions and transitions
- updated resistor lookup table
- Corrected while in e_stop test, so it monitors for button press instead of button release
- removed some unused code
- changed update_averages to take average voltage from voltage divider
- Comments to help identify different blocks of variables
- Updated optimize Pitch
*************************************************************************************************************/
#include <LiquidCrystal_I2C.h> // Uses a zip library downloaded from https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

// ARDUINO PINS
#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
#define NACELLE_SWITCH_PIN 4
#define LOAD_SWITCH_PIN 12
#define LOAD_NET_SWITCH_1_PIN 46
#define LOAD_NET_SWITCH_2_PIN 47
#define LOAD_NET_SWITCH_4_PIN 48
#define LOAD_NET_SWITCH_8_PIN 49
#define ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN 13
#define BRAKE_PIN 8
#define EBRAKE_BTN_PIN 18
#define ACTUATOR_PIN 5
#define VOLTAGE_PIN A12

// ACTUATOR/SERVO POSITIONS
#define ACTUATOR_MAX 130
#define ACTUATOR_MIN 60
#define INITIAL_PITCH 100
#define INITIAL_LOAD 12
#define BRAKE_PITCH 63    // This value is arbitrary, needs to be tested
#define BRAKE_ENGAGED 100
#define BRAKE_DISENGAGED 125

#define BASE_RESISTOR 0.5 // Value of the resistor at the end of the load, acts as last resistor in a voltage divider

#define LOAD_DISCONNECT_THRESHOLD 0.05 

#define STEADY_POWER_RANGE_PERCENT 0.05

// VALUES FOR TRANSITION TO AND USE IN POWER CURVE STATE
#define POWER_CURVE_RPM 1235.0
#define POWER_CURVE_VOLTAGE 5.0
#define PWR_CURVE_CONST_THRESHOLD 150.0 
#define PWR_CURVE_PERCENT_THRESHOLD 1.1

LiquidCrystal_I2C lcd(0x3F, 20, 4);  

const int sampleSize = 20;
float sampledRPM[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledVoltage[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledPower[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float sampledWindSpeeds[sampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int windSpeed = 4;
int current_angle = 0;

float avgRPM = 0.0;
float prevRPM = 0.0;
float avgVoltage = 0.0;
float avgPower = 0.0; 

int currResistorIndex = INITIAL_LOAD;
float voltage_factor;

float pwr_at_11 = 0.0;
float rpm_at_11 = 0.0;

float TARGET_MAX_PWR;
float TARGET_MIN_PWR;
float TARGET_MAX_RPM;

bool externalPower = true;

bool is_brake_engaged = false;

Servo pitchControl;
Servo brakeControl;

Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

// These are the total resistance values through the resistor network and over the base resistor
float resistorLookupTable[16] = {10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 18.0};
//          indices -->           0     1     2      3    4     5      6     7     8    9     10    11    12    13    14    15

float resistorRPMPowerLUT[3][7] = {  
   {  5.0,   6.0,    7.0,   8.0,      9.0,   10.0,   11.0 }, // m/s 
   {  12,     8,      6,     12,      12,      0,     9   }, // resistor index
   {1400.0, 1630.0, 1940.0, 2300.0, 2550.0, 2880.0, 3180.0} // rpm
};

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
    optimize_pitch_test,
    brake_test,
    ebrake_test,
    encoder_test,
    voltage_test,
    wind_speed_test,
    switch_test,
    load_switch_test,
    load_disconnect_test,
    toggle_test,
    wait
} test_type;

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
    pinMode(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    
    digitalWrite(LOAD_SWITCH_PIN, HIGH);
    digitalWrite(NACELLE_SWITCH_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_8_PIN, LOW);

    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, HIGH);
    delay(2000);

    pitchControl.write(INITIAL_PITCH);
    pitchControl.attach(ACTUATOR_PIN);

    brakeControl.write(BRAKE_DISENGAGED);
    brakeControl.attach(BRAKE_PIN);
    
    delay(2000);
    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, LOW);
    
    currentState = restart;
    oldState = restart;

    Serial.write("Starting up the turbine!\n");

    // set_pitch(INITIAL_PITCH);
    // delay(2000);

    internal_power_switch();
    set_load(INITIAL_LOAD);
}


void loop() {

    test_type test_state = get_input();

    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        if (test_state == toggle_test) {
            // toggle_power_switch();
//            set_pitch(INITIAL_PITCH);
            // toggle_power_switch();
            Serial.println("Entering RESTART State!");
            currentState = restart;
        } else {
            delay(1000);
        }
        break;
    case restart:
        if (test_state == toggle_test) {
            Serial.println("Testing!");
            currentState = testing;
        } else if (is_E_stop() || !is_load_connected()) {
            Serial.println("Entering ESTOP State!");
            currentState = emergency_shutdown;
        // } else if (avgRPM >= POWER_CURVE_RPM && avgVoltage >= POWER_CURVE_VOLTAGE) {
        } else if (avgRPM >= POWER_CURVE_RPM) {
            Serial.println("Entering POWER_CURVE State!");
            currResistorIndex = resistorRPMPowerLUT[1][0];
            set_load(currResistorIndex);
            currentState = power_curve;
        }
        break;
    case power_curve:
        if (test_state == toggle_test) {
          Serial.println("Testing!");
            currentState = testing;
        } else if (is_E_stop() || !is_load_connected()) {
            Serial.println("Entering ESTOP State!");
            currentState = emergency_shutdown;
        }

        // Monitor for increase in RPM
        if (avgRPM >= TARGET_MAX_RPM && windSpeed == 11){
            Serial.println("Entering STEADY_POWER State!");
            TARGET_MAX_PWR = pwr_at_11 + pwr_at_11 * STEADY_POWER_RANGE_PERCENT;
            TARGET_MIN_PWR = pwr_at_11 - pwr_at_11 * STEADY_POWER_RANGE_PERCENT;
            TARGET_MAX_RPM = prevRPM - prevRPM * STEADY_POWER_RANGE_PERCENT;
            currentState = steady_power;
        }
        break;
    case steady_power:
        if (test_state == toggle_test) {
            Serial.println("Testing!");
            currentState = testing;
        } else if (is_E_stop() || !is_load_connected()) {
            Serial.println("Entering ESTOP State!");
            currentState = emergency_shutdown;
        }
        break;
    case emergency_shutdown:
        if (test_state == toggle_test) {
            Serial.println("Testing!");
            currentState = testing;
        }
        if (!is_E_stop() && is_load_connected()) {
            Serial.println("Entering RESTART State!");
            set_pitch(INITIAL_PITCH);
            set_load(INITIAL_LOAD);
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
        switch (test_state) {
        case pitch_test:
            test_pitch();
            break;
        case calibrate_test:
            calibrate_pitch();
            break;
        case optimize_pitch_test:
            while (!Serial.available()) {
              optimize_pitch();
              delay(1000);
            }
            break;
        case brake_test:
            Serial.println("Testing BRAKE");
            digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, HIGH);
            delay(1000);
            if (is_brake_engaged) {
              Serial.println("Testing disengage");
              brakeControl.write(BRAKE_DISENGAGED);
              is_brake_engaged = false;
            } else {
              Serial.println("Testing engage");              
              brakeControl.write(BRAKE_ENGAGED);
              is_brake_engaged = true;
            }
            delay(3000);
            digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, LOW);
            break;
        case ebrake_test:
            Serial.println("Testing EBRAKE");
            while (!is_E_stop());
            digitalWrite(LOAD_SWITCH_PIN, HIGH);
            digitalWrite(NACELLE_SWITCH_PIN, LOW);
            externalPower = true;
            delay(1000);
            engage_ebrake();
            break;
        case load_disconnect_test:
            Serial.println("Testing Load Disconnect");
            while (is_load_connected());
            delay(1000);
            engage_ebrake();
            break;
        case encoder_test:
            Serial.println(encoder());
            break;
        case voltage_test:
            Serial.println(analogRead(VOLTAGE_PIN));
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
        if (avgRPM > TARGET_MAX_RPM) {
            windSpeed++;
            char strBuf[50];
            sprintf(strBuf, "Wind speed increased to %d m/s", windSpeed);
            Serial.println(strBuf);
            // TARGET_MAX_RPM = resistorRPMPowerLUT[2][windSpeed - 5] * PWR_CURVE_PERCENT_THRESHOLD;
            // TARGET_MAX_RPM = resistorRPMPowerLUT[2][windSpeed - 5] + PWR_CURVE_CONST_THRESHOLD;
            TARGET_MAX_RPM = resistorRPMPowerLUT[2][windSpeed - 5];
            
            currResistorIndex = resistorRPMPowerLUT[1][windSpeed - 5];
            set_load(currResistorIndex);
        
            // optimize_load(); For now we are just setting it to the tested resistor value
            rpm_at_11 = avgRPM;
            pwr_at_11 = avgPower;
        }
        break;

    case steady_power:
        optimize_pitch();
        break;

    case emergency_shutdown:
        break;
    }

    // HANDLE ENCODER AND WIND SPEED
    encoder();
    update_averages();
    write_to_lcd(); // When running in competition, comment out this line of code to increase processing speed
    
}

void optimize_pitch() {

  if (avgPower < TARGET_MIN_PWR) {
    Serial.println("Increasing Pitch");
    current_angle = pitchControl.read();
    set_pitch(current_angle+1);
  }

  if (avgPower > TARGET_MAX_PWR || avgRPM > TARGET_MAX_RPM) {
    Serial.println("Decreasing Pitch");
    current_angle = pitchControl.read();
    set_pitch(current_angle-1);
  }

}

// void optimize_load() {

//     float powerVals[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     for (int resistor = 0; resistor < 15; resistor++) {
//       set_load(resistor);
//       delay(1000);
//       for (int i = 0; i < sampleSize; i++){
//         update_averages();
//       }
//       powerVals[resistor] = avgPower;
//     }
//     int max_power_resistor = 0;
//     float max_power_val = 0.0;
//     for (int i = 0; i < 15; i++) {
//       if (powerVals[i] > max_power_val) {
//         max_power_resistor = i;
//       }
//     } 
//     set_load(max_power_resistor);
     
// }

// void optimize_load_v2(){

//     float best_pwr_val = avgPower;
//     bool isIncreasing = true;

//     // test power value as we increase resistance
//     while (currResistorIndex < 15 && isIncreasing){

//         // test new resistor value
//         currResistorIndex++;
//         set_load(currResistorIndex);
//         for (int i = 0; i < sampleSize; i++){
//             update_averages();
//         }

//         if (avgPower > best_pwr_val * 1.05){
//             best_pwr_val = avgPower;
//         }
//         else {
//             currResistorIndex--;
//             set_load(currResistorIndex);
//             isIncreasing = false;
//         }
//     }

//     // test power value as we decrease resistance
//     while (currResistorIndex > 0 && isIncreasing){

//         // test new resistor value
//         currResistorIndex--;
//         set_load(currResistorIndex);
//         for (int i = 0; i < sampleSize; i++){
//             update_averages();
//         }

//         if (avgPower > best_pwr_val * 1.05){
//             best_pwr_val = avgPower;
//         }
//         else {
//             currResistorIndex++;
//             set_load(currResistorIndex);
//             isIncreasing = false;
//         }
//     }
// }

void set_pitch(int pitch_angle) {
  
    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, HIGH);
    delay(1000);
    current_angle = pitchControl.read();
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
    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, LOW);
}

void engage_ebrake() {
  
    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, HIGH);
    delay(1000);
    current_angle = pitchControl.read();
    if (ACTUATOR_MIN < current_angle) {
        for (int pos = current_angle-1; pos >= ACTUATOR_MIN; pos--) {
            pitchControl.write(pos);
            delay(50);
        }
    }
    if (ACTUATOR_MIN > current_angle) {
        for (int pos = current_angle+1; pos <= ACTUATOR_MIN; pos++) {
            pitchControl.write(pos);
            delay(50);
        }
    }

    brakeControl.write(BRAKE_ENGAGED);
    is_brake_engaged = true;
    
    delay(2000);
    digitalWrite(ACTUATOR_BRAKE_SWITCH_SIGNAL_PIN, LOW);
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

void set_load(int binary_val) {
    if (binary_val > 15 || binary_val < 0) {
        Serial.println("Invalid load value");
    } else {
        currResistorIndex = binary_val;
        if (binary_val & 0x1) {
            digitalWrite(LOAD_NET_SWITCH_1_PIN, HIGH);
        } else {
            digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
        }
        if (binary_val & 0x2) {
            digitalWrite(LOAD_NET_SWITCH_2_PIN, HIGH);
        } else {
            digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
        }
        if (binary_val & 0x4) {
            digitalWrite(LOAD_NET_SWITCH_4_PIN, HIGH);
        } else {
            digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);
        }
        if (binary_val & 0x8) {
            digitalWrite(LOAD_NET_SWITCH_8_PIN, HIGH);
        } else {
            digitalWrite(LOAD_NET_SWITCH_8_PIN, LOW);
        }
    }
    set_voltage_factor();
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
    int voltage = analogRead(VOLTAGE_PIN);
    return voltage != 0;
}

bool is_E_stop() {
     return digitalRead(EBRAKE_BTN_PIN);
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
         if (!strcmp(inputString, "ebrake")) {
             return ebrake_test;
         }
         if (!strcmp(inputString, "load")) {
             return load_disconnect_test;
         }
         if (!strcmp(inputString, "rpm")) {
             return encoder_test;
         }
         if (!strcmp(inputString, "volts")) {
             return voltage_test;
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
         if (!strcmp(inputString, "calib")) {
             return calibrate_test;
         }
         if (!strcmp(inputString, "opt")) {
             return optimize_pitch_test;
         }
     }
     return wait;
 }

// Recalculate the avgWindSpeed, update the sampleWindSpeed array pushing out the oldest data point
// and adding the newest data point
// void update_avg_wind_speed(float measuredSpeed) {
//     avgWindSpeed = ((avgWindSpeed * sampleSize) - sampledWindSpeeds[0] + measuredSpeed) / sampleSize;
    
//     for (int i = 0; i < (sampleSize - 1); i++) {
//         sampledWindSpeeds[i] = sampledWindSpeeds[i + 1];
//     }
    
//     sampledWindSpeeds[sampleSize - 1] = measuredSpeed;
// }

void update_avg_rpm(float measuredRPM) {
    avgRPM = ((avgRPM * sampleSize) - sampledRPM[0] + measuredRPM) / sampleSize;
    for (int i = 0; i < sampleSize - 1; i++) {
        sampledRPM[i] = sampledRPM[i + 1];
    }
    sampledRPM[sampleSize - 1] = measuredRPM;
}

void set_voltage_factor() {
    voltage_factor = (resistorLookupTable[currResistorIndex] / BASE_RESISTOR);
}
 
void update_averages() {
    float voltage = (analogRead(VOLTAGE_PIN) / 1023.0) * 5.0 * voltage_factor; 
 
    float resistance = resistorLookupTable[currResistorIndex];

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
    float resistance = resistorLookupTable[currResistorIndex];
    lcd.print(resistance);
    lcd.setCursor(15, 0);
    lcd.print("P:");
    lcd.setCursor(17, 0);
    lcd.print(avgPower);

    // Print RPM and pitch
    //int pitch_in_degrees = actuator_to_degrees(current_angle);
    lcd.setCursor (0, 1);            
    lcd.print("RPM:");
    lcd.setCursor (4, 1);
    lcd.print(avgRPM);
    lcd.setCursor (10, 1);
    lcd.print((char)0xF2);
    lcd.setCursor(11, 1);
    lcd.print(":");
    lcd.setCursor(12, 1);
    lcd.print(current_angle);

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

// int actuator_to_degrees(int actuator_value){
//   int degree = 90 * (actuator_value - ACTUATOR_MIN) / (ACTUATOR_MAX - ACTUATOR_MIN);
//   return degree;
// }

// int degrees_to_actuator(int degrees){
//   int actuator_value = ACTUATOR_MIN + (degrees / 90) * (ACTUATOR_MAX - ACTUATOR_MIN);
//   return actuator_value;
// }
