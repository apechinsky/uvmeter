#include <SPI.h>
#include <LowPower.h>
#include <VoltageReference.h>

#include "Screen.h"

int UV_OUT_PIN = A0; 
int UV_EN_PIN = 11; 

int SHAKE_PIN = 2;

int LED_PIN = 13;

int WAKE_UP_PIN = 2;

volatile unsigned long lastTiltTime = 0;

int IDLE_TIME = 30 * 1000;

Screen screen;

VoltageReference voltage;

void setup() {
    screen.begin();
    voltage.begin(1.128 * 1000 * 1023);

    Serial.begin(9600);

    pinMode(UV_OUT_PIN, INPUT);
    pinMode(UV_EN_PIN, INPUT);

    pinMode(LED_PIN, OUTPUT);

    pinMode(SHAKE_PIN, INPUT);

    pinMode(WAKE_UP_PIN, INPUT);   
}


void loop() {

    if (millis() - lastTiltTime > IDLE_TIME) {
        screen.off();
        uvEnable(true);

        sleepTillShake();

        uvEnable(false);
        screen.on();
    }

    measure();

    blink(1000);
}

void uvEnable(bool enable) {
    digitalWrite(UV_EN_PIN, enable ? HIGH : LOW);
}

void blink(int duration) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
}

void sleepTillShake() {

    attachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN), wakeUp, CHANGE);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    /* detachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN));  */
}

void wakeUp() {
    lastTiltTime = millis();
}

void measure() {
    int uvLevel = averageAnalogRead(UV_OUT_PIN);
    double uvVoltage = getUvVoltage(uvLevel);
    double uvIntensity = getUvIntensity(uvVoltage);

    screen.show(uvLevel, uvIntensity, voltage.readVcc() / (double)1000);

    printState(uvLevel, uvVoltage, uvIntensity);
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

