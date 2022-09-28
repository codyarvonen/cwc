// https://github.com/arduino-libraries/Servo
#include <Servo.h>
#define ACTUATOR_PIN 12

Servo pitchControl;

void setup() {
    Serial.begin(9600);
    pitchControl.attach(ACTUATOR_PIN);
}

void loop() {
    // if (Serial.available() > 0) {
    //     // char *input = Serial.readString();
    //     // int pitch = atoi(input);
    //     int pitch = Serial.read();
    //     // char temp[8];
    //     // sprinf(temp, "%d\n", pitch);
    //     // Serial.write(temp);
    //     if (pitchControl.attached()) {
    //         if (pitch == '0') {
    //           Serial.println("0");
    //             set_pitch(0);
    //         }
    //         if (pitch == '1') {
    //           Serial.println("50");
    //             set_pitch(50);
    //         }
    //         if (pitch == '2') {
    //           Serial.println("100");
    //             set_pitch(100);
    //         } else {
    //           Serial.println("150");
    //             set_pitch(150);
    //         }
    //     } else {
    //         Serial.println("Actuator is not attached.");
    //     }
    // }

    // TODO: SET BOUNDARY MAX AND MIN FOR ACTUATOR

    set_pitch(0);
    set_pitch(50);
    set_pitch(100);
    set_pitch(150);
    set_pitch(100);
    set_pitch(50);
    set_pitch(0);
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
