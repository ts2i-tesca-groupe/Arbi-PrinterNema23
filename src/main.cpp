#include <AccelStepper.h>
#include <Arduino.h>

#define MEASURE millis();
// #define MEASURE micros();
#define MICROSTEPS  1600

const int ena = 2;  // enable pin
const int dir = 3;  // direction pin
const int pul = 4;  // pulse pin
const int motorInterfaceType = AccelStepper::DRIVER;
unsigned long mtime[4];

unsigned long nsteps = 0;
unsigned long nrev = 1;
AccelStepper myStepper(motorInterfaceType, pul, dir);

void setup() {
    Serial.begin(9600);
    pinMode(ena, OUTPUT);
    digitalWrite(ena, LOW);  // Activer le driver


    myStepper.setAcceleration(100000);
    myStepper.setMaxSpeed(1600000);
    nrev = 1;
}

void loop() {
    nsteps++;
    myStepper.moveTo(nrev * MICROSTEPS);
    mtime[0] = MEASURE
    myStepper.runToPosition();
    mtime[1] = MEASURE
    
    delay(1000);
    myStepper.moveTo(0);
    mtime[2] = MEASURE

    myStepper.runToPosition();
    mtime[3] = MEASURE

    Serial.print(nsteps);
    Serial.print("-->\t[");
    Serial.print(mtime[0]);Serial.print(",\t");
    Serial.print(mtime[1]);Serial.print("]\t");
    Serial.print(mtime[1] - mtime[0]);Serial.print("\t\t*****\t\t[");
    Serial.print(mtime[2]);Serial.print(",\t");
    Serial.print(mtime[3]);Serial.print("]\t");
    Serial.print(mtime[3] - mtime[2]);
    Serial.print("\t");
    Serial.println(static_cast<float>(((mtime[3] - mtime[2])*1000)/(nrev * MICROSTEPS)));

    delay(1000);
}