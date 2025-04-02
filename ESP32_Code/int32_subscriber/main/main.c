#include <Arduino.h>

const int servoPin = 2; // Change this to the actual GPIO pin connected to your servo
const int freq = 50; // Servo PWM frequency
const int channel = 0; // LEDC channel
const int resolution = 16; // Resolution of PWM (16-bit)

void setup() {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(servoPin, channel);
}

void setServoAngle(int angle) {
    int dutyCycle = map(angle, 0, 270, 1638, 8192); // Convert angle to duty cycle (500-2500us)
    ledcWrite(channel, dutyCycle);
}

void loop() {
    setServoAngle(0); // Move to 0 degrees
    delay(5000); // Wait 5 seconds

    setServoAngle(270); // Move to 270 degrees
    delay(5000); // Wait 5 seconds
}
