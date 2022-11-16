// https://github.com/arduino-libraries/Servo
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>
#include <string.h>

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3
#define ACTUATOR_PIN 5
#define WALL_SWITCH_PIN 10
#define GEN_LOAD_SWITCH_PIN 11

#define EBRAKE_BTN_PIN 8
#define BRAKE_SERVO_PIN 6
#define LOAD_READ_PIN 4
#define LOAD_NET_SWITCH_1_PIN 7
#define LOAD_NET_SWITCH_2_PIN 12
#define LOAD_NET_SWITCH_4_PIN 9

#define ENGAGED_BRAKE_ANGLE 90
#define DISENGAGED_BRAKE_ANGLE 110
#define ACTUATOR_MAX 80
#define ACTUATOR_MIN 40
#define LOAD_DISCONNECT_THRESHOLD 0.1

bool brakeIsEngaged = false;
bool wallPower = true;

Servo pitchControl;
Servo brakeControl;

Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

enum states_t {
    restart,
    power_curve,
    steady_power,
    survival,
    emergency_shutdown,
    testing
} currentState;

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
    pinMode(GEN_LOAD_SWITCH_PIN, OUTPUT);
    pinMode(WALL_SWITCH_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_1_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_2_PIN, OUTPUT);
    pinMode(LOAD_NET_SWITCH_4_PIN, OUTPUT);

    digitalWrite(GEN_LOAD_SWITCH_PIN, LOW);
    digitalWrite(WALL_SWITCH_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_1_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_2_PIN, LOW);
    digitalWrite(LOAD_NET_SWITCH_4_PIN, LOW);

    delay(1000);

    pitchControl.write(ACTUATOR_MAX);
    pitchControl.attach(ACTUATOR_PIN);
    brakeControl.write(DISENGAGED_BRAKE_ANGLE);
    brakeControl.attach(BRAKE_SERVO_PIN);

    currentState = restart;

    Serial.write("Beginning test! \n");
}

void loop() {

    test_type test_state = get_input();

    /***********************/
    /** STATE TRANSITIONS **/
    /***********************/
    switch (currentState) {
    case testing:
        if (test_state == toggle_test) {
            currentState = restart;
        }
        break;
    case restart:
        if (test_state == toggle_test) {
            currentState = testing;
        } else if (!load_connected() || digitalRead(EBRAKE_BTN_PIN)) {
            currentState = emergency_shutdown;
        }
        // set pitch to startup angle, trigger switches back
        break;
    case power_curve:
        if (test_state == toggle_test) {
            currentState = testing;
        } else if (!load_connected() || digitalRead(EBRAKE_BTN_PIN)) {
            currentState = emergency_shutdown;
        }
        /* Figure out how to determine wind speed
        if(avgWindSpeed >= 11 m/s){
            currentState = steady_power;
        }
        */
        break;
    case steady_power:
        if (test_state == toggle_test) {
            currentState = testing;
        } else if (!load_connected() || digitalRead(EBRAKE_BTN_PIN)) {
            currentState = emergency_shutdown;
        }
        /* Figure out how to determine wind speed
        if(avgWindSpeed > 14 m/s){
            currentState = survival;
        }
        */
        break;
    case survival:
        if (test_state == toggle_test) {
            currentState = testing;
        } else if (!load_connected() || digitalRead(EBRAKE_BTN_PIN)) {
            currentState = emergency_shutdown;
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in intial position?
        break;
    case emergency_shutdown:
        if (test_state == toggle_test) {
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
        switch (test_state) {
        case pitch_test:
            test_pitch();
            break;
        case brake_test:
            if (brakeIsEngaged) {
                Serial.println("Disengaging brake");
                disengageBrake();
            } else {
                Serial.println("Engaging brake");
                engageBrake();
            }
            break;
        case encoder_test:
            Serial.println(encoder());
            break;
        case wind_speed_test:
            Serial.println("Wind speed test method not yet implemented");
            break;
        case switch_test:
            Serial.println("Toggle switches");
//            digitalWrite(GEN_LOAD_SWITCH_PIN, wallPower);
//            digitalWrite(WALL_SWITCH_PIN, !wallPower);

            digitalWrite(GEN_LOAD_SWITCH_PIN, !digitalRead(GEN_LOAD_SWITCH_PIN));
            digitalWrite(WALL_SWITCH_PIN, !digitalRead(WALL_SWITCH_PIN));
            
            wallPower = !wallPower;
            break;
        case load_switch_test:
            test_load_switches();
            break;
        case wait:
            Serial.println("Waiting for input");
            break;
        default:
            Serial.println("Something unexpected has occured!");
        }
        break;

    // Pitch blades out to inital state, trigger switches back so generator powers
    // nacelle components and the load
    case restart:
        break;

    // Adjust the load to reach maximum Cp at each wind speed 5-11 m/s
    case power_curve:
        break;

    // Adjust the pitch to maintain constant power output from 11-14 m/s
    case steady_power:
        break;

    // Adjust pitch and load to minimize rpm and stress in the system
    case survival:
        break;

    // Flip switches so components draw from the wall and the generator runs
    // to the brake circuit
    case emergency_shutdown:
        break;
    }

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

void engageBrake() { brakeControl.write(ENGAGED_BRAKE_ANGLE); }

void disengageBrake() { brakeControl.write(DISENGAGED_BRAKE_ANGLE); }

float encoder() {
    static long oldPosition = 0;
    static long oldtime = 0;
    float rpm = 0;

    long newPosition = myEnc.read();

    if (newPosition != oldPosition) {
        long newtime = micros();
        float dx = (newPosition - oldPosition);
        float dt = newtime - oldtime;
        oldPosition = newPosition;
        oldtime = newtime;
        rpm = (dx * 1000000 * 60) / (dt * 2048);
    }
    return rpm;
}

bool load_connected() {
    float voltage = analogRead(LOAD_READ_PIN) * 5.0 / 1023.0;
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
    Serial.println(pitch);
//    if (pitch < ACTUATOR_MIN) {
//        pitch = ACTUATOR_MIN;
//    }
//    if (pitch > ACTUATOR_MAX) {
//        pitch = ACTUATOR_MAX;
//    }
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
        char incomingByte;
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
    Serial.println("Checking for user input");
    if (Serial.available() > 0) {
        Serial.println("User input has been detected!");
        char inputString[7];
        for (int i = 0; i < 6; i++) {
            inputString[i] = Serial.read();
            if (inputString[i] == '\n') {
              
                inputString[i + 1] = '\0';
                break;
            }
        }
        clear_buf();
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
        if (!strcmp(inputString, "swich\n")) {
            return switch_test;
        }
    }
    return wait;
}
