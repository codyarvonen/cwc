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
#define ACTUATOR_MIN 40
#define LOAD_DISCONNECT_THRESHOLD 0.1

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
}

// unsigned int integerValue = 0;
// char incomingByte;
int pitch;

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
        // set pitch to startup angle, trigger switches back
        break;
    case power_curve:
        if (toggle_test()) {
            currentState = testing;
        }
        /* Figure out how to determine wind speed
        if(avgWindSpeed >= 11 m/s){
            currentState = steady_power;
        }
        */
        check_E_Stop();
        break;
    case steady_power:
        if (toggle_test()) {
            currentState = testing;
        }
        /* Figure out how to determine wind speed
        if(avgWindSpeed > 14 m/s){
            currentState = survival;
        }
        */
        check_E_Stop();
        break;
    case survival:
        if (toggle_test()) {
            currentState = testing;
        }
        // Set load and pitch to values that are best fit for survival
        // same as E_Stop but leave switches in intial position?
        check_E_Stop();
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

void check_E_Stop() {
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
        currentState = emergency_shutdown;
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