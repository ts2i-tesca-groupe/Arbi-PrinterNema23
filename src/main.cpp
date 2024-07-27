#include <Arduino.h>
#include <MagicPot.h>

#define DEBUG_BAUDIOS       115200
#define POTENTIOMETER_PIN   A1
#define MIN_VAL_CHANGE      5
#define MAX_SPEED           500000

MagicPot potentiometer(POTENTIOMETER_PIN, 0, MAX_SPEED, MAX_RAW_VALUE_DEFAULT);

uint16_t oldSpeed = 0;
void speedUpdate(){
    if (abs(potentiometer.getRawValue() - oldSpeed) > MIN_VAL_CHANGE){
        /* update speed */
        oldSpeed = potentiometer.getRawValue();
    }
    
}

void setup()
{
	Serial.begin(DEBUG_BAUDIOS);

	potentiometer.begin();
}

void loop()
{
	potentiometer.read();

    delay(1000);
}