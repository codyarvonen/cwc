// https://github.com/arduino-libraries/Servo

/************************************************************************************************************
TESTS TO RUN
1 - Run tests with sampledRPM and sampledWindSpeeds to make sure the avgRPM and avgWindSpeed update
correctly 2 - Test ability to sense load disconnect and enter emergency shutdown 3 - Test ability to
sense when load is reconnected, it will help to know how low of an RPM we can still sense a voltage
across the load, and if we should change the resistor load to help with this. 4 - Test resistor
relay, make sure that switches lead to different load resistances.
*************************************************************************************************************/
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
#define NACELLE_SWITCH_PIN 4
#define LOAD_SWITCH_PIN 5
#define LOAD_NET_SWITCH_1_PIN 7
#define LOAD_NET_SWITCH_2_PIN 8
#define LOAD_NET_SWITCH_4_PIN 9
#define ACTUATOR_SWITCH_SIGNAL_PIN 10
#define EBRAKE_BTN_PIN 11
#define ACTUATOR_PIN 12
#define VOLTAGE_PIN 18
#define ESTOP_LED 19
#define STEADY_POWER_LED 20
#define POWER_CURVE_LED 21
#define SURVIVAL_LED 22

#define ACTUATOR_MAX 100
#define ACTUATOR_MIN 0
#define INITIAL_PITCH 30 // This value is arbitrary, needs to be tested
#define BRAKE_PITCH 0    // This value is arbitrary, needs to be tested

#define LOAD_DISCONNECT_THRESHOLD 0.05
#define STEADY_POWER_RPM 2500.0
#define STEADY_POWER_RANGE 100.0

// These are not ohm values, but rather which resistor configuration #1-8
const int lookupResistorTable[15] = {1, 2, 3, 4, 5, 6, 7, 8, 6, 6, 6, 6, 6, 6, 6};
// Pair of arrays acting as resistor lookup value given wind speed
const int lookupWindSpeedTable[15] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

float avgWindSpeed = 0.0;
float sampledWindSpeeds[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float avgRPM = 0.0;
float sampledRPM[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool externalPower = true;

Servo pitchControl;

Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

enum states_t {
    restart,
    power_curve,
    steady_power,
    survival,
    emergency_shutdown,
    testing
} currentState;

enum states_s { primary_switch_state, secondary_switch_state } switchState;

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
    pinMode(LOAD_SWITCH_PIN, OUTPUT);
    pinMode(NACELLE_SWITCH_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_1_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_2_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_4_PIN, OUTPUT);
    pinMode(ACTUATOR_SWITCH_SIGNAL_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(ESTOP_LED, OUTPUT);
    pinMode(POWER_CURVE_LED, OUTPUT);
    pinMode(STEADY_POWER_LED, OUTPUT);
    pinMode(SURVIVAL_LED, OUTPUT);

    digitalWrite(LOAD_SWITCH_PIN, LOW);
    digitalWrite(NACELLE_SWITCH_PIN, HIGH);
    digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);

    pitchControl.write(INITIAL_PITCH);
    pitchControl.attach(ACTUATOR_PIN);

    currentState = restart;

//    set_state_led();

    Serial.write("Starting up the turbine!\n");
}

int pitch;
int resistorValue = 0;
bool restartProtocolComplete = false;
int currentPitch = 0;

void loop() {

    test_type test_state = get_input();

    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        if (test_state == toggle_test) {
            toggle_power_switch();
            set_pitch(INITIAL_PITCH);
            toggle_power_switch();
            currentState = restart;
        } else {
            delay(1000);
        }
        break;
    case restart:
        if (test_state == toggle_test) {
            currentState = testing;
//            set_state_led();
        } else if (is_load_connected()) {
            currentState = power_curve;
//            set_state_led();
        }
        break;
    case power_curve:
        if (test_state == toggle_test) {
            currentState = testing;
//            set_state_led();
        } else if (is_E_stop()) {
            currentState = emergency_shutdown;
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if (avgWindSpeed >= 11) {
            currentState = steady_power;
//            set_state_led();
        }
        break;
    case steady_power:
        if (test_state == toggle_test) {
            currentState = testing;
//            set_state_led();
        } else if (is_E_stop()) {
            currentState = emergency_shutdown;
//            set_state_led();
        }
        // Figure out how to determine wind speed, lookup table or windspeed sensor
        if (avgWindSpeed > 14) {
            currentState = survival;
//            set_state_led();
        } else if (avgWindSpeed < 11) {
            currentState = power_curve;
//            set_state_led();
        }

        break;
    case survival:
        if (test_state == toggle_test) {
            currentState = testing;
//            set_state_led();
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in initial position?
        if (avgWindSpeed < 14) {
            currentState = steady_power;
//            set_state_led();
        }
        if (is_E_stop()) {
            currentState = emergency_shutdown;
//            set_state_led();
        }
        break;
    case emergency_shutdown:
        if (test_state == toggle_test) {
            currentState = testing;
//            set_state_led();
        }
        if (!is_E_stop()) {
            toggle_power_switch();
            set_pitch(INITIAL_PITCH);
            toggle_power_switch();
            currentState = restart;
//            set_state_led();
        }
        break;
    }

    /********************/
    /** STATE ACTIONS ***/
    /********************/

    switch (currentState) {

    // This mode is designed for testing with manual control
    case testing:
        Serial.println("Testing");
        switch (test_state) {
        case pitch_test:
            test_pitch();
            break;
        case brake_test:
            Serial.println("Testing EBRAKE");
            while (is_E_stop());
            externalPower = true;
            digitalWrite(LOAD_SWITCH_PIN, externalPower);
            digitalWrite(NACELLE_SWITCH_PIN, !externalPower);
            delay(1000);
            set_pitch(60);
        case encoder_test:
            Serial.println(encoder());
            break;
        case wind_speed_test:
            break;
        case switch_test:
            toggle_power_switch();
            break;
        // case load_switch_test:
        //     test_load_switches();
        //     break;
        case wait:
            break;
        }
        break;

    // Pitch blades out to inital state, trigger switches back so generator powers
    // nacelle components and the load
    case restart:
        break;

    // Adjust the load to reach maximum Cp at each wind speed 5-11 m/s
    // TODO: test ideal resistor values for varying wind speeds and fixed pitch angles
    case power_curve:
//        resistorValue = lookupResistor(avgWindSpeed);
//        set_load_resistance(resistorValue); // set this load based on current power Cp? Current wind
                                            // speed? current rpm?
        // TODO: Create function that uses lookup table(s) to determine and set resistor value
        break;

    // Adjust the pitch to maintain constant power output from 11-14 m/s
    // TODO: test pitch angles to have a fixed rpm for varying wind speeds and set resistor value
    case steady_power:
//        int curr_rpm = 0;
//        currentPitch = 0;
////        set_load_resistance(6); // move this so it is set right before moving to steady_power?
//        curr_rpm = encoder();   // Get a rolling average instead of single data point
//        if (curr_rpm < STEADY_POWER_RPM - STEADY_POWER_RANGE) {
//            currentPitch = pitchControl.read() + 5; // Find a good step size
//            set_pitch(currentPitch);                // set this pitch based on current rpm
//        } else if (curr_rpm > STEADY_POWER_RPM + STEADY_POWER_RANGE) {
//            currentPitch = pitchControl.read() - 5; // Find a good step size
//            set_pitch(currentPitch);                // set this pitch based on current rpm
//        }

        break;

    // Adjust pitch and load to minimize rpm and stress in the system
    case survival:
        break;

    // After pitch is set to brake state, pitch is set to a the braking angle, this state is help
    // until load is connected and button disengaged
    case emergency_shutdown:
        set_pitch(BRAKE_PITCH);
        break;
    }

    // HANDLE ENCODER AND WIND SPEED
//    sample_wind_speed();
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

void toggle_power_switch() {
    digitalWrite(LOAD_SWITCH_PIN, externalPower);
    digitalWrite(NACELLE_SWITCH_PIN, !externalPower);
    externalPower = !externalPower;
}

//void set_state_led() {
//    digitalWrite(ESTOP_LED, LOW);
//    digitalWrite(POWER_CURVE_LED, LOW);
//    digitalWrite(STEADY_POWER_LED, LOW);
//    digitalWrite(SURVIVAL_LED, LOW);
//    switch (currentState) {
//    case testing:
//        digitalWrite(ESTOP_LED, HIGH);
//        digitalWrite(POWER_CURVE_LED, HIGH);
//        digitalWrite(STEADY_POWER_LED, HIGH);
//        digitalWrite(SURVIVAL_LED, HIGH);
//        break;
//    case restart:
//        digitalWrite(ESTOP_LED, HIGH);
//        digitalWrite(POWER_CURVE_LED, HIGH);
//        break;
//    case power_curve:
//        digitalWrite(POWER_CURVE_LED, HIGH);
//        break;
//    case steady_power:
//        digitalWrite(STEADY_POWER_LED, HIGH);
//        break;
//    case survival:
//        digitalWrite(SURVIVAL_LED, HIGH);
//        break;
//    case emergency_shutdown:
//        digitalWrite(ESTOP_LED, HIGH);
//        break;
//    }
//}
void set_load(int binary_val) {
    if (binary_val >= 8 || binary_val < 0) {
        Serial.println("Invalid load value");
    } else {
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
    }
}

// void set_switches(bool initialState) {
//     /* The initial state has the load switch and generator to nacelle switch closed, and the wall
//     to nacelle switch open. The generator to nacelle switch and the load switch share a digital
//     signal. The wall to nacelle switch and the brake switch also share a digital signal. */

//     // First set all switches to open, to avoid ever having all switches closed
//     digitalWrite(PRIMARY_SWITCH_SIGNAL_PIN, LOW);
//     digitalWrite(SECONDARY_SWITCH_SIGNAL_PIN, LOW);
//     if (initialState) {
//         digitalWrite(PRIMARY_SWITCH_SIGNAL_PIN, HIGH);
//     } else {
//         digitalWrite(SECONDARY_SWITCH_SIGNAL_PIN, HIGH);
//     }
// }

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

//    update_avg_rpm(rpm);
    return rpm;
}

bool is_load_connected() {
    float voltage = analogRead(VOLTAGE_PIN) * 5.0 / 1023.0;
    if (voltage < LOAD_DISCONNECT_THRESHOLD) {
        return false;
    }
    return true;
}

bool is_E_stop() {
     return !digitalRead(EBRAKE_BTN_PIN);

//    if (digitalRead(EBRAKE_BTN_PIN)) {
//        return true;
//    }
//    return !is_load_connected();
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
         if (!strcmp(inputString, "rpm")) {
             return encoder_test;
         }
         if (!strcmp(inputString, "wind")) {
             return wind_speed_test;
         }
         if (!strcmp(inputString, "switch")) {
             return switch_test;
         }
     }
     return wait;
 }

 
//void update_avg_rpm(float measuredRPM) {
//    avgRPM = ((avgRPM * sizeof(sampledRPM)) - sampledRPM[0] + measuredRPM) / sizeof(sampledRPM);
//    for (int i = 0; i < sizeof(sampledWindSpeeds) - 1; i++) {
//        sampledRPM[i] = sampledRPM[i + 1];
//    }
//    sampledRPM[sizeof(sampledRPM) - 1] = measuredRPM;
//}

// Recalculate the avgWindSpeed, update the sampleWindSpeed array pushing out the oldest data point
// and adding the newest data point
//void update_avg_wind_speed(float measuredSpeed) {
//    avgWindSpeed =
//        ((avgWindSpeed * sizeof(sampledWindSpeeds)) - sampledWindSpeeds[0] + measuredSpeed) /
//        sizeof(sampledWindSpeeds);
//    for (int i = 0; i < (sizeof(sampledWindSpeeds) - 1); i++) {
//        sampledWindSpeeds[i] = sampledWindSpeeds[i + 1];
//    }
//    sampledWindSpeeds[sizeof(sampledWindSpeeds) - 1] = measuredSpeed;
//}

//void sample_wind_speed() {
//    // Sample the pitot tube and calculate the wind speed, or use a lookup table using rpm and pitch
//    // angle
//    float currentWindSpeed = 5.0;
//    update_avg_wind_speed(currentWindSpeed);
//}

//int lookupResistor(float avgWindSpeed) {
//    int roundedWindSpeed = (int)(avgWindSpeed + 0.5);
//    int index = 0;
//    for (int i = 0; i < sizeof(lookupWindSpeedTable); i++) {
//        if (lookupWindSpeedTable[i] == roundedWindSpeed) {
//            index = i;
//        }
//    }
//    return lookupResistorTable[index];
//}
