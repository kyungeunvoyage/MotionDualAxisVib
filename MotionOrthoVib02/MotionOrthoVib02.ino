#include <Wire.h>
#include "Haptic_Driver.h"

//=========================DA7280============================
#define NUMBER_OF_SENSORS 2
Haptic_Driver hapDrive[NUMBER_OF_SENSORS];
hapticSettings hf;
#define MUX_ADDR 0x70
bool daInitialized[NUMBER_OF_SENSORS];

int burst75ms = 75;
int burst200ms = 200;
int burst500ms = 500;

float currentFreq = 10.0;
bool vibrating = false;
unsigned long vibStart = 0;
unsigned long vibDuration = 0;
//================DA7280====================

//================setup the MUX and DA7280====================
void TCA9548A(uint8_t bus) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

void DA7280setup()
{
    hf.motorType = LRA_TYPE;
    hf.absVolt = 3.5;
    hf.nomVolt = 2.47;
    hf.currMax = 295.1;
    hf.impedance = 8.37;
    //init frequency 
    hf.lraFreq = 5;
    Wire.begin();

    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
        TCA9548A(i);
        hapDrive[i].begin();
        daInitialized[i] = true;
        if (!hapDrive[i].begin()) {
            Serial.print("Could not communicate with Haptic Driver ");
            Serial.println(i);
        }
        hapDrive[i].enableFreqTrack(false);
        hapDrive[i].setOperationMode(DRO_MODE);
    }
}
//================setup the MUX and DA7280====================




// The setup() function runs once each time the micro-controller starts
void setup()
{
    Serial.begin(115200);
    Serial.println("Hello Hello");
    DA7280setup();

}

// Add the main program code into the continuous loop() function
void loop()
{


}
