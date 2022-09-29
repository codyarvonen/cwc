// https://github.com/arduino-libraries/Servo
#include <Servo.h>
#include <math.h>

#define ACTUATOR_PIN 12
#define MAX_PITCH_CHARS 4
#define MAX_PITCH_SIZE 3

int digits = 0;

Servo pitchControl;

void setup() {
    Serial.begin(9600);
    pitchControl.attach(ACTUATOR_PIN);
}

void loop() {

    char pitches[MAX_PITCH_CHARS];

    while (Serial.available() > 0) {
        Serial.println("Getting digit");
        if (digits < MAX_PITCH_CHARS) {
            pitches[digits] = Serial.read();
            digits++;
        }
    }
    if (digits > 0) {
        Serial.println(digits);
    }
    if (digits > 1) {
        Serial.println(digits);
        for (int i = 0; i < digits; i++) {
            Serial.println(pitches[i]);
        }
        int pitch = (pitches[0] - 48) * pow(10, (digits - 2));
        Serial.println(pitch);
        if (digits > 2) {
            pitch += (pitches[1] - 48) * pow(10, (digits - 3));
        }
        Serial.println(pitch);
        if (digits > 3) {
            pitch += (pitches[2] - 48) * pow(10, (digits - 4));
        }
        Serial.println(pitch);
        if (pitchControl.attached()) {
            set_pitch(pitch);
        } else {
            Serial.println("Actuator is not attached.");
        }
        digits = 0;
    }

    // TODO: SET BOUNDARY MAX AND MIN FOR ACTUATOR
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