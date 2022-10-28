// https://github.com/arduino-libraries/Servo
#include <Encoder.h> //Uses Encoder library by Paul Stoffregen v 1.4.2
#include <Servo.h>

#define ACTUATOR_PIN 12
#define BRAKE_SERVO_PIN 9

const int ENGAGED_BRAKE_ANGLE = 90;
const int DISENGAGED_BRAKE_ANGLE = 110;
const int ACTUATOR_MAX = 100;
const int ACTUATOR_MIN = 40;

bool brakeIsEngaged = false;

Servo pitchControl;
Servo brakeControl;

//   Change the two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);

// Used for the encoder
long oldPosition = -999;
long oldtime = 0;
float dx;
float dt;
float cps;
float rps;
float old_rps;
float delta_rps;

void setup() {
    Serial.begin(9600);
    Serial.write("Beginning test! \n");
    pitchControl.attach(ACTUATOR_PIN);
    brakeControl.attach(BRAKE_SERVO_PIN);
}

unsigned int integerValue = 0;
char incomingByte;
int pitch;
const int OutPin = A0;  // wind sensor analog pin  hooked up to Wind P sensor "OUT" pin
const int TempPin = A2; // temp sesnsor analog pin hooked up to Wind P sensor "TMP" pin

void loop() {

    int windADunits = analogRead(OutPin);
    float windMPH = pow((((float)windADunits - 264.0) / 85.6814), 3.36814);
    Serial.print(windMPH);
    Serial.print(" MPH\t");
    int tempRawAD = analogRead(TempPin);
    float tempC = ((((float)tempRawAD * 5.0) / 1024.0) - 0.400) / .0195;
    Serial.print(tempC);
    Serial.println(" C");

    encoder();
    delay(1000);

    engageBrake();

    encoder();
    delay(1000);

    disengageBrake();

    encoder();
    delay(1000);

    set_pitch(45);

    encoder();
    delay(1000);

    set_pitch(80);

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
    delay(15);
}

void engageBrake() { brakeControl.write(ENGAGED_BRAKE_ANGLE); }

void disengageBrake() { brakeControl.write(DISENGAGED_BRAKE_ANGLE); }

void encoder() {
    long newPosition = myEnc.read();

    if (newPosition != oldPosition) {
        long newtime = micros();
        dx = (newPosition - oldPosition);
        dt = newtime - oldtime;
        oldPosition = newPosition;
        oldtime = newtime;
        cps = dx / (dt / 1000000);
        rps = cps / 2048;
        delta_rps = rps - old_rps;
        old_rps = rps;
        Serial.println(rps * 60);
    }
}
