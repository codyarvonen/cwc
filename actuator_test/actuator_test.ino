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
        int pitch = Serial.read();
        Serial.write(pitch);
        set_pitch(pitch);
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
    current_angle = pitch_angle;
    delay(15);
}
