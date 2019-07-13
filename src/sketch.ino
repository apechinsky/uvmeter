#include <SPI.h>
#include <LowPower.h>
#include <VoltageReference.h>

#include "Screen.h"

// #define DEBUG

// Calibration value should be obtained individually for each 
// controller instance
const int VOLTAGE_CALIBRATION = 1.128 * 1000 * 1023;

int UV_OUTPUT = A0; 

int UV_ENABLE_PIN = 11; 

int TILT_SENSOR_PIN = 2;

int IDLE_TIME = 10 * 1000;

volatile unsigned long lastTiltTime = 0;


Screen screen;

VoltageReference voltage;

void setup() {
    screen.begin();

    #ifdef DEBUG
    Serial.begin(9600);
    #endif

    voltage.begin(VOLTAGE_CALIBRATION);

    pinMode(UV_OUTPUT, INPUT);
    pinMode(UV_ENABLE_PIN, INPUT);
    pinMode(TILT_SENSOR_PIN, INPUT);   
}


void loop() {

    if (millis() - lastTiltTime > IDLE_TIME) {
        sleepTillShake();
    }

    measureAndShow();

    delay(1000);
}

void sleepTillShake() {
    screen.off();
    uvEnable(true);

    attachInterrupt(digitalPinToInterrupt(TILT_SENSOR_PIN), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

    uvEnable(false);
    screen.on();
}

void uvEnable(bool enable) {
    digitalWrite(UV_ENABLE_PIN, enable ? HIGH : LOW);
}

void wakeUp() {
    lastTiltTime = millis();
}

void measureAndShow() {
    int uvLevel = averageAnalogRead(UV_OUTPUT);
    double uvVoltage = getUvVoltage(uvLevel);
    double uvIntensity = getUvIntensity(uvVoltage);

    screen.show(uvLevel, uvIntensity, voltage.readVcc() / (double)1000);

    #ifdef DEBUG
    printState(uvLevel, uvVoltage, uvIntensity);
    #endif
}

void printState(int uvLevel, double uvVoltage, double uvIntensity) {
    Serial.print(" VCC: ");
    Serial.print(voltage.readVcc());

    Serial.print(" UV level: ");
    Serial.print(uvLevel);

    Serial.print(" / UV v: ");
    Serial.print(uvVoltage);

    Serial.print(" / UV (mW/cm^2): ");
    Serial.print(uvIntensity);

    Serial.print(" time: ");
    Serial.print(lastTiltTime);

    Serial.println();
}

//
// Convert UV digital output to voltage
//
double getUvVoltage(int uvLevel) {
    // Method 1
    // Sparkfun recommends to use regulated 3.3 arduino output as base voltage
    // Wiring: Arduino 3.3 - Arduino A0
    // int refLevel = averageAnalogRead(REF_3V3);
    // double uvVoltage = 3.3 / refLevel * uvLevel;

    // Method 2
    // We use internal 1.1 voltage as a base because battery voltage
    // may be in range 2.8-4.0 and so we do not have regulated 3.3v
    int refLevel = voltage.readInternalRef();

    double uvVoltage = 1.1 / refLevel * uvLevel;

    return uvVoltage;
}

//
// Convert the voltage to a UV intensity level
//
double getUvIntensity(double uvVoltage) {
    double uvIntensity = mapDouble(uvVoltage, 0.99, 2.8, 0.0, 15.0); 

    return  max(0, uvIntensity);
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead) {
    byte numberOfReadings = 8;
    unsigned int runningValue = 0; 

    for (int x = 0 ; x < numberOfReadings ; x++) {
        runningValue += analogRead(pinToRead);
    }

    runningValue /= numberOfReadings;

    return(runningValue);  
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

