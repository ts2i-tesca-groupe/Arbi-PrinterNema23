#include <AccelStepper.h>
#include <Arduino.h>
#include <MagicPot.h>

// #define MEASURE millis();
#define MEASURE         micros();
#define MICROSTEPS      400
#define MAXLENGTH       600
#define ZEROPOS         20
#define NBCYCLES        1


#define SPEED           1500
#define ACCEL           SPEED / 4

#define DEBUG_BAUDIOS       115200
#define POTENTIOMETER_PIN   A1
#define MIN_VAL_CHANGE      5
#define MAX_SPEED           500000


// Pins declaration
const uint8_t ena = 2;  // enable pin
const uint8_t dir = 3;  // direction pin
const uint8_t pul = 4;  // pulse pin
const uint8_t motorInterfaceType = AccelStepper::DRIVER;
const uint8_t home = 5;
const uint8_t rightlimit = 6;
const uint8_t startcyc = 7;
const uint8_t selectorpins[4] = {8, 9, 10, 11};

const size_t ncycles[4] = {1, 3, 5, 8};
const float lcycles[4] = {439, 142.33, 100.75, 52.5};
const float p = 40.05;

unsigned long mtime[4];
unsigned long nsteps = 0;
unsigned long nrev = 1;
bool problem = false;
bool homedetected = false;
MagicPot potentiometer(POTENTIOMETER_PIN, 0, MAX_SPEED, MAX_RAW_VALUE_DEFAULT);

uint16_t oldSpeed = 0;
void speedUpdate(){
    if (abs(potentiometer.getRawValue() - oldSpeed) > MIN_VAL_CHANGE){
        /* update speed */
        oldSpeed = potentiometer.getRawValue();
    }
    
}

size_t i = 0;

AccelStepper myStepper(motorInterfaceType, pul, dir);

unsigned long mm2steps(float mm) {
    return mm * (MICROSTEPS / p);
}

uint8_t getCycle() {
    return 3;
    uint8_t pos = 0;
    for (size_t i = 0; i < 4; i++) {
        pos += (i + 1) * digitalRead(selectorpins[i]);
    }

    problem = pos == 0 || pos > 4;
    return --pos;
}

void goHome() {
    myStepper.setAcceleration(ACCEL);
    myStepper.setMaxSpeed(SPEED);
    //myStepper.moveTo(-mm2steps(MAXLENGTH));
    Serial.println("Goging home");
    return;
    while (myStepper.isRunning()) {
        myStepper.run();
        Serial.println(myStepper.currentPosition());
        if (digitalRead(home) == HIGH) {
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
    Serial.println("At zero pos");
}

void setup() {
    Serial.begin(DEBUG_BAUDIOS);
	potentiometer.begin();
    pinMode(home, INPUT);
    pinMode(rightlimit, INPUT);
    pinMode(startcyc, INPUT);
    for (size_t j = 0; j < 4; j++) {
        pinMode(selectorpins[j], INPUT);
    }

    myStepper.setPinsInverted(false, false, true);
    myStepper.setEnablePin(ena);

    goHome();
    Serial.println(myStepper.currentPosition());
    nrev = 1;
}

void loop() {
    potentiometer.read();
    if (digitalRead(startcyc) && !problem) {
        Serial.print("Starting : ACCEL = ");
        Serial.print(ACCEL);
        Serial.print(" SPEED = ");
        Serial.print(SPEED);
        Serial.print(" MICROSTEPS = ");
        Serial.println(MICROSTEPS);
        int ncycle = getCycle();
        for (; !problem && !digitalRead(home) && i < NBCYCLES; i++) {
            //Serial.println(mm2steps(lcycles[ncycle]) * nrev);
            myStepper.move(mm2steps(lcycles[ncycle]) * nrev);
            mtime[0] = MEASURE
            while (myStepper.isRunning()) {
                if (false && digitalRead(rightlimit)) {
                    Serial.println("Limit reached rightlimit");
                    myStepper.stop();
                    // myStepper.disableOutputs();
                    // problem = true;
                    break;
                }
                myStepper.run();
            }
            mtime[1] = MEASURE
            //return;
            //Serial.println(myStepper.currentPosition());
            //Serial.println(-(signed)mm2steps(lcycles[ncycle] * nrev));
            myStepper.move(-mm2steps(lcycles[ncycle] * nrev));
            mtime[2] = MEASURE
            /*
            while (myStepper.isRunning() && !problem) {
                if (digitalRead(home)) {
                    Serial.println("Limit reached home");
                    myStepper.stop();
                    // myStepper.disableOutputs();
                    // problem = true;
                    break;
                }
                myStepper.run();
            }
            mtime[3] = MEASURE
            */
            //Serial.println(myStepper.currentPosition());
            //Serial.println(String("i = ") + i);

            Serial.print(nsteps++);
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
        }
    }
    

    // nsteps++;
    /*myStepper.run();
    if (digitalRead(5) == HIGH) {
        Serial.println("Dicht");
        myStepper.moveTo(1275);
    }

    if (digitalRead(6) == HIGH) {
        Serial.println("Open");
        myStepper.moveTo(0);
    }

    if (digitalRead(7) == HIGH) {
        Serial.println("Stop");
        myStepper.stop();
    }
    /*
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
    */
}