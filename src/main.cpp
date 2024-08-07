#include <AccelStepper.h>
#include <Arduino.h>
#include <MagicPot.h>

// #define MEASURE millis();
#define MEASURE micros();
#define MICROSTEPS 400
#define MAXLENGTH 600
#define ZEROPOS 20
#define NBCYCLES 1

#define SPEED_HOME 1000
#define ACCEL_HOME 750

#define DEBUG_BAUDIOS 115200
#define POTENTIOMETER_PIN A1
#define MIN_VAL_CHANGE 5
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

const size_t ncycles[4] = {50, 3, 5, 8};
const float lcycles[4] = {400, 142.33, 100.75, 52.5};  // 439
const float p = 40.05;

unsigned long mtime[4];
unsigned long nsteps = 0;
unsigned long nrev = 10;
bool problem = false;
bool homedetected = false;
MagicPot potentiometer(POTENTIOMETER_PIN, 10, MAX_SPEED, MAX_RAW_VALUE_DEFAULT);

uint16_t oldSpeed = 0;
void speedUpdate() {
    if (abs(potentiometer.getRawValue() - oldSpeed) > MIN_VAL_CHANGE) {
        /* update speed */
        oldSpeed = potentiometer.getRawValue();
    }
}

AccelStepper myStepper(motorInterfaceType, pul, dir);

unsigned long mm2steps(float mm) {
    return mm * (MICROSTEPS / p);
}

uint8_t getCycle() {
    uint8_t pos = 0;
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
int cpt = 0;
int tnb0[7] = {0, 0, 0, 0, 0, 0, 0};
void goHome() {
    // return;
    myStepper.setAcceleration(ACCEL_HOME);
    myStepper.setMaxSpeed(SPEED_HOME);
    myStepper.moveTo(-mm2steps(MAXLENGTH));
    Serial.println(String("Goging home ") + myStepper.targetPosition());
    int nb0 = 0, newnb = 0;
    while (myStepper.isRunning()) {
        myStepper.run();
        tnb0[0] += !digitalRead(startcyc);
        tnb0[1] += !digitalRead(home);
        tnb0[2] += !digitalRead(rightlimit);
        tnb0[3] += !digitalRead(selectorpins[0]);
        tnb0[4] += !digitalRead(selectorpins[1]);
        tnb0[5] += !digitalRead(selectorpins[2]);
        tnb0[6] += !digitalRead(selectorpins[3]);

        if (cpt % 100 == 0) {
            Serial.println(myStepper.currentPosition() + 
                String(" cpt1 : ") + cpt +
                String(" startcyc : ") + tnb0[0] + String(" ") +
                String("Home : ") + tnb0[1] + String(" ") +
                String("rightlimit : ") + tnb0[2] + String(" ") +
                tnb0[3] + String(" ") + tnb0[4] + String(" ") + tnb0[5] +
                String(" ") + tnb0[6]);
        }
        cpt++;
        continue;
        bool status = digitalRead(home);
        nb0 += !status;
        if (nb0 != newnb) {
            Serial.println(myStepper.currentPosition() + String(" Home: ") + status + String(" nb0: ") + nb0);
            newnb = nb0;
        }
        // if (!digitalRead(home)) {
        //     Serial.println("Home");
        //     myStepper.stop();
        //     myStepper.setCurrentPosition(0);
        //     homedetected = true;
        //     break;
        // }
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
    // potentiometer.onChange(speedUpdate, false);
    pinMode(home, INPUT_PULLUP);
    pinMode(rightlimit, INPUT_PULLUP);
    pinMode(startcyc, INPUT_PULLUP);
    for (size_t j = 0; j < nbpins; j++) {
        pinMode(selectorpins[j], INPUT_PULLUP);
    }

    myStepper.setPinsInverted(false, false, true);
    myStepper.setEnablePin(ena);

    goHome();
    Serial.println(myStepper.currentPosition());
    nrev = 1;
}

void loop() {
    return;

    tnb0[0] += !digitalRead(startcyc);
    tnb0[1] += !digitalRead(home);
    tnb0[2] += !digitalRead(rightlimit);
    tnb0[3] += !digitalRead(selectorpins[0]);
    tnb0[4] += !digitalRead(selectorpins[1]);
    tnb0[5] += !digitalRead(selectorpins[2]);
    tnb0[6] += !digitalRead(selectorpins[3]);

    if (cpt % 500 == 0) {
        Serial.println(String("cpt : ") + cpt +
                       String("startcyc : ") + tnb0[0] + String(" ") +
                       String("Home : ") + tnb0[1] + String(" ") +
                       String("rightlimit : ") + tnb0[2] + String(" ") +
                       tnb0[3] + String(" ") + tnb0[4] + String(" ") + tnb0[5] +
                       String(" ") + tnb0[6]);
    }
    cpt++;
    return;

    Serial.println(String("startcyc : ") + digitalRead(startcyc) + String(" ") +
                   String("Home : ") + digitalRead(home) + String(" ") +
                   String("rightlimit : ") + digitalRead(rightlimit) + String(" "));
    for (size_t i = 0; i < nbpins; i++) {
        Serial.print(String(" ") + digitalRead(selectorpins[i]));
    }
    Serial.println();
    potentiometer.read();
    Serial.println(String("Potentio : ") + potentiometer.getValue());
    // Serial.println(String("Home : ") + digitalRead(home) + String(" ") +
    // String("rightlimit : ") + digitalRead(rightlimit) + String(" "));
    // delay(1500);
    return;
    if (digitalRead(startcyc) && !problem) {
        Serial.print("Starting : ACCEL = ");
        Serial.print(ACCEL_HOME);
        Serial.print(" SPEED = ");
        Serial.print(SPEED_HOME);
        Serial.print(" MICROSTEPS = ");
        Serial.println(MICROSTEPS);
        int ncycle = getCycle();
        if (ncycle == 255) return;
        for (size_t i = 0; !problem && digitalRead(home) && i < ncycles[ncycle]; i++) {
            // Serial.println(mm2steps(lcycles[ncycle]) * nrev);
            uint16_t currSpeed = potentiometer.getValue();
            myStepper.setAcceleration(currSpeed / DEVISER);
            myStepper.setMaxSpeed(currSpeed);
            Serial.println(String("maxSpeed: ") + currSpeed +
                           String("\tAccel: ") + currSpeed / DEVISER);
            myStepper.move(mm2steps(lcycles[ncycle]) * nrev);
            mtime[0] = MEASURE while (myStepper.isRunning()) {
                if (false && !digitalRead(rightlimit)) {
                    Serial.println("Limit reached rightlimit");
                    myStepper.stop();
                    // myStepper.disableOutputs();
                    // problem = true;
                    break;
                }
                myStepper.run();
                potentiometer.read();
            }
            mtime[1] = MEASURE
                // return;
                // Serial.println(myStepper.currentPosition());
                // Serial.println(-(signed)mm2steps(lcycles[ncycle] * nrev));
                currSpeed = potentiometer.getValue();
            myStepper.setAcceleration(currSpeed / DEVISER);
            myStepper.setMaxSpeed(currSpeed);
            Serial.println(String("------maxSpeed: ") + currSpeed +
                           String("\tAccel: ") + currSpeed / DEVISER);
            myStepper.move(-mm2steps(lcycles[ncycle] * nrev));
            mtime[2] = MEASURE

                while (myStepper.isRunning() && !problem) {
                if (!digitalRead(home)) {
                    Serial.println("Limit reached home");
                    myStepper.stop();
                    // myStepper.disableOutputs();
                    // problem = true;
                    break;
                }
                myStepper.run();
                potentiometer.read();
            }
            mtime[3] = MEASURE
                           /**/
                           // Serial.println(myStepper.currentPosition());
                           // Serial.println(String("i = ") + i);

                           Serial.print(nsteps++);
            Serial.print("-->\t[");
            Serial.print(mtime[0]);
            Serial.print(",\t");
            Serial.print(mtime[1]);
            Serial.print("]\t");
            Serial.print(mtime[1] - mtime[0]);
            Serial.print("\t");
            Serial.print(static_cast<float>(((mtime[1] - mtime[0]) * 1000 / MICROSTEPS)));
            Serial.print("\t*****\t[");
            Serial.print(mtime[2]);
            Serial.print(",\t");
            Serial.print(mtime[3]);
            Serial.print("]\t");
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