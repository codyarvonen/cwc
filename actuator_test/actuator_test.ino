// https://github.com/arduino-libraries/Servo
#include <Servo.h>

#define ACTUATOR_PIN 12

Servo pitchControl;

void setup() {
    Serial.begin(9600);
    pitchControl.attach(ACTUATOR_PIN);
}

void loop() {
    if (Serial.available() > 0) {
        int pitch = atoi(Serial.readString());
        // int pitch = Serial.read();
        char temp[8];
        sprinf(temp, "%d\n", pitch);
        Serial.write(temp);
        if (pitchControl.attached()) {
            set_pitch(pitch);
        } else {
            Serial.println("Actuator is not attached.");
        }
    }
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
