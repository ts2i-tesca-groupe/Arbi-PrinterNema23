#include <Arduino.h>
#include <AccelStepper.h>

const int ena = 2;          // enable pin
const int dir = 3;          // direction pin
const int pul = 4;          // pulse pin
const int motorInterfaceType = 1;

AccelStepper myStepper(motorInterfaceType, pul, dir);


void setup() {
    pinMode(ena, OUTPUT);
    digitalWrite(ena, LOW);   // Activer le driver

    myStepper.setAcceleration(5000);
    myStepper.setMaxSpeed(3200);
}

void loop() {
    myStepper.moveTo(100000);
    myStepper.runToPosition();
    delay(1000);
    myStepper.moveTo(0);
    myStepper.runToPosition();
    delay(1000);
}