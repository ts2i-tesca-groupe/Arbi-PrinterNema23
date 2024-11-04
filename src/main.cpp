#include <AccelStepper.h>
#include <Arduino.h>
#include <MagicPot.h>

// #define MEASURE millis();
#define MEASURE micros();
#define MICROSTEPS 400
#define MAXLENGTH 300
#define ZEROPOS 20
#define NBCYCLES 1

#define SPEED_HOME 500
#define ACCEL_HOME 750

#define DEBUG_BAUDIOS 115200
#define POTENTIOMETER_PIN A1
// Update this line to set the max speed
#define MAX_SPEED 20000
#define DEVISER 0.5

// Pins declaration
const uint8_t ena = 2;  // enable pin
const uint8_t dir = 3;  // direction pin
const uint8_t pul = 4;  // pulse pin
const uint8_t motorInterfaceType = AccelStepper::DRIVER;
const uint8_t home = 5;
const uint8_t rightlimit = 6;
const uint8_t startcyc = 7;
const uint8_t nbpins = 5;
const uint8_t selectorpins[nbpins] = {8, 9, 10, 11, 12};
const uint8_t printer = 13;

const size_t ncycles[4] = {1, 3, 4, 8};
// Update this line according to different lenghts (in mm)
const float lcycles[4] = {400, 142.33, 100.75, 52.5};  // 439
const float p = 40.05;

const unsigned long activeSteps = 20;
bool problem = false;
bool homedetected = false;
MagicPot potentiometer(POTENTIOMETER_PIN, 10, MAX_SPEED, MAX_RAW_VALUE_DEFAULT);

AccelStepper myStepper(motorInterfaceType, pul, dir);

unsigned long mm2steps(float mm) {
    return mm * (MICROSTEPS / p);
}

short getCycle() {
    short pos = 0;
    for (size_t i = 0; i < nbpins; i++) {
        pos += (i + 1) * !digitalRead(selectorpins[i]);
        /*Serial.print(String("getCycle() i = ") + i + String(" !digitalRead(selectorpins[i] = ") + !digitalRead(selectorpins[i]) + String(" pos = ") + pos);*/
    }
    Serial.println(String("getCycle() pos = ") + pos);

    if (pos == 0 || pos > 4) {
        return -1;
    }
    return --pos;
}

void goHome() {
    // return;
    myStepper.setAcceleration(ACCEL_HOME);
    myStepper.setMaxSpeed(SPEED_HOME);
    myStepper.moveTo(-mm2steps(MAXLENGTH));
    Serial.println(String("Goging home ") + myStepper.targetPosition());

    while (myStepper.isRunning()) {
        myStepper.run();
        bool status = digitalRead(home);
        Serial.println(myStepper.currentPosition() + String(" Home: ") + status);
        // continue;
        if (!status) {
            Serial.println("Home");
            myStepper.stop();
            myStepper.setCurrentPosition(0);
            homedetected = true;
            break;
        }
    }
    if (!homedetected) {
        myStepper.disableOutputs();
        problem = true;
        return;
    }

    Serial.println("At home");
    myStepper.runToNewPosition(mm2steps(ZEROPOS));
    myStepper.setCurrentPosition(0);
    Serial.println("At zero pos");
}

void setup() {
    Serial.begin(DEBUG_BAUDIOS);
    potentiometer.begin();
    // potentiometer.onChange(speedUpdate, false);
    pinMode(home, INPUT_PULLUP);
    pinMode(rightlimit, INPUT_PULLUP);
    pinMode(startcyc, INPUT_PULLUP);
    for (size_t j = 0; j < nbpins; j++) {
        pinMode(selectorpins[j], INPUT_PULLUP);
    }
    pinMode(printer, OUTPUT);
    digitalWrite(printer, LOW);

    myStepper.setPinsInverted(false, false, true);
    myStepper.setEnablePin(ena);

    goHome();
    Serial.println(myStepper.currentPosition());
}

bool printOn = false;
bool printOff = false;

void loop() {
    if (!digitalRead(startcyc) && !problem) {
        // Serial.print("Starting : ACCEL = ");
        // Serial.print(ACCEL_HOME);
        // Serial.print(" SPEED = ");
        // Serial.print(SPEED_HOME);
        // Serial.print(" MICROSTEPS = ");
        // Serial.println(MICROSTEPS);
        int ncycle = getCycle();
        // Serial.println(String("ncycle: ") + ncycle);
        // delay(500);
        // return;
        if (ncycle < 0) return;
        potentiometer.read();
        uint16_t currSpeed = potentiometer.getValue();
        unsigned long nsteps = mm2steps(lcycles[ncycle]);
        long oldpos = -1;
        long currpos = 0;
        myStepper.setAcceleration(currSpeed / DEVISER);
        myStepper.setMaxSpeed(currSpeed);
        Serial.println(String("maxSpeed: ") + currSpeed +
                       String("\tAccel: ") + currSpeed / DEVISER +
                       String(" heading for ") + nsteps * ncycles[ncycle] +
                       String(" nsteps ") + nsteps);

        myStepper.move(nsteps * ncycles[ncycle]);
        // Moving forward
        while (myStepper.isRunning()) {
            // Serial.println(myStepper.currentPosition());
            if (!digitalRead(rightlimit)) {
                Serial.println("Limit reached rightlimit");
                myStepper.stop();
                myStepper.disableOutputs();
                problem = true;
                break;
            }
            currpos = myStepper.currentPosition();
            if (currpos % nsteps == 0) {
                if(currpos != oldpos){
                    printOn = true;
                    oldpos = currpos;
                }
            }
            // if (((myStepper.currentPosition() + nsteps)%(nsteps + activeSteps)) == 0){
            if ((currpos % nsteps) == activeSteps) {
                if(currpos != oldpos){
                    printOff = true;
                    oldpos = currpos;
                }
            }
            // Active the printer
            if (printOn) {
                digitalWrite(printer, HIGH);
                printOn = false;
                Serial.println(String("Printer On at ") + myStepper.currentPosition());
            }
            // Reset the signal
            if (printOff) {
                digitalWrite(printer, LOW);
                printOff = false;
                Serial.println(String("Printer Off at ") + myStepper.currentPosition());
            }
            // Serial.println(String("current: ") + myStepper.currentPosition() +
            //                 String(" target: ") + myStepper.targetPosition() +
            //                 String(" distanceToGo: ") + myStepper.distanceToGo());
            myStepper.run();
        }
        myStepper.setAcceleration(MAX_SPEED);
        myStepper.move(-nsteps * ncycles[ncycle]);
        while (myStepper.isRunning() && !problem) {
            if (!digitalRead(home)) {
                Serial.println("Limit reached home");
                myStepper.stop();
                myStepper.disableOutputs();
                problem = true;
                break;
            }
            myStepper.run();
        }
    }
}