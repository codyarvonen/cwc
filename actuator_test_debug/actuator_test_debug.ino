// https://github.com/arduino-libraries/Servo
#include <Servo.h>

#define ACTUATOR_PIN 12

#define ACTUATOR_MIN 0
#define ACTUATOR_MAX 200

Servo pitchControl;

void setup() {
    Serial.begin(9600);
    pitchControl.attach(ACTUATOR_PIN);
}

unsigned int integerValue = 0;
char incomingByte;
int pitch;
int currPitch = 0;
int prevPitch = 0;

void loop() {

    if (Serial.available() > 0) { // something came across serial
        integerValue = 0;         // throw away previous integerValue
        while (1) {               // force into a loop until '\n' is received
            incomingByte = Serial.read();
            if (incomingByte == '\n')
                break; // exit the while(1), we're done receiving
            if (incomingByte == -1)
                continue;       // if no characters are in the buffer read() returns -1
            integerValue *= 10; // shift left 1 decimal place
            // convert ASCII to integer, add, and shift left 1 decimal place
            integerValue = ((incomingByte - 48) + integerValue);
        }
        // Do something with the value
        pitch = integerValue;
        if (pitchControl.attached()) {
            if (ACTUATOR_MIN <= pitch && pitch <= ACTUATOR_MAX) {
                currPitch = pitch;
                Serial.print("Setting pitch to ");
                Serial.println(pitch);
                set_pitch(pitch);
            } else {
                Serial.println("Invalid Input");
            }
        } else {
            Serial.println("Actuator is not attached.");
        }
    }
}

void set_pitch(int pitch_angle) {
    int current_angle = pitchControl.read();
    Serial.println("current pitch:");
    Serial.println(current_angle);
    Serial.println("actual current pitch:");
    Serial.println(prevPitch);
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
    prevPitch = currPitch;
    delay(15);
}
