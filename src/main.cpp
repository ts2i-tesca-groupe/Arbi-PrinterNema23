#include <AccelStepper.h>
#include <Arduino.h>

// #define MEASURE millis();
#define MEASURE micros();
#define MICROSTEPS  400

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
    myStepper.setPinsInverted(false, false, true);
    myStepper.setEnablePin(ena);
    /*pinMode(ena, OUTPUT);
    digitalWrite(ena, LOW);  // Activer le driver*/

    myStepper.setAcceleration(100000);
    myStepper.setMaxSpeed(16000);
    myStepper.setCurrentPosition(0);
    nrev = 10;
}

void loop() {
    nsteps++;
    //myStepper.move(nrev * (MICROSTEPS));
    myStepper.setCurrentPosition(0);
    mtime[0] = MEASURE
    myStepper.runToNewPosition(nrev * (MICROSTEPS));
    //myStepper.runToPosition();
    mtime[1] = MEASURE
    



    delay(500);
    //myStepper.moveTo(0);
    myStepper.setCurrentPosition(0);
    mtime[2] = MEASURE
    myStepper.runToNewPosition(-nrev * (MICROSTEPS));
    //myStepper.runToPosition();
    mtime[3] = MEASURE

    Serial.print(nsteps);
    Serial.print("-->\t[");
    Serial.print(mtime[0]);Serial.print(",\t");
    Serial.print(mtime[1]);Serial.print("]\t");
    Serial.print(mtime[1] - mtime[0]);Serial.print("\t");
    Serial.print(static_cast<float>(((mtime[1] - mtime[0]) * 1000 / MICROSTEPS)));
    Serial.print("\t*****\t[");
    Serial.print(mtime[2]);Serial.print(",\t");
    Serial.print(mtime[3]);Serial.print("]\t");
    Serial.print(mtime[3] - mtime[2]);
    Serial.print("\t");
    Serial.print(static_cast<float>((1000 * 1000 / (mtime[3] - mtime[2]))));
    Serial.print("\t");
    Serial.println(static_cast<float>(((mtime[3] - mtime[2]) * 1000 / MICROSTEPS)));

    delay(500);
}