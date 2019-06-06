#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LowPower.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
/* #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin) */
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
int TILT_PIN = 2;
int LED_PIN = 13;

int WAKE_UP_PIN = 2;


void setup() {
    Serial.begin(9600);

    pinMode(UVOUT, INPUT);
    pinMode(REF_3V3, INPUT);

    pinMode(LED_PIN, OUTPUT);

    pinMode(TILT_PIN, INPUT);

    /* pinMode(WAKE_UP_PIN, INPUT_PULLUP);    */
    pinMode(WAKE_UP_PIN, INPUT);   

    setupDisplay();
}

void loop() {

    sleepDisplay();

    sleepTillShake();

    wakeDisplay();

    blink(2000);

    measure();

    blink(2000);
}

void blink(int duration) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
}

void setupDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
    }
}

void sleepDisplay() {
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void wakeDisplay() {
  display.ssd1306_command(SSD1306_DISPLAYON);
}

void sleepTillShake() {

    attachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN), wakeUp, CHANGE);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    detachInterrupt(digitalPinToInterrupt(WAKE_UP_PIN)); 
}

void wakeUp() {
}

void measure() {
    int uvLevel = averageAnalogRead(UVOUT);
    int refLevel = averageAnalogRead(REF_3V3);

    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    double outputVoltage = 3.3 / refLevel * uvLevel;
    double inputVoltage = 3.3 / refLevel * 1023;

    /* double outputVoltage = 5.0 / refLevel * uvLevel; */
    /* double inputVoltage = 5.0 / refLevel * 1023; */

    double uvIntensity = mapDouble(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

    Serial.print(" Arduino power: ");
    Serial.print(inputVoltage);

    Serial.print(" 3v3 level: ");
    Serial.print(refLevel);

    Serial.print(" ML8511 level: ");
    Serial.print(uvLevel);

    Serial.print(" / ML8511 V: ");
    Serial.print(outputVoltage);

    Serial.print(" / UV (mW/cm^2): ");
    Serial.print(uvIntensity);

    Serial.println();

    displayValues(uvIntensity, inputVoltage, uvLevel);
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead) {
    byte numberOfReadings = 8;
    unsigned int runningValue = 0; 

    for(int x = 0 ; x < numberOfReadings ; x++)
        runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;

    return(runningValue);  
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void sleep(unsigned int seconds) {
    while (seconds >= 8) {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        seconds -=8; 
    }
    if (seconds >= 4) {
        LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
        seconds -=4; 
    }
    if (seconds >= 2) {
        LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
        seconds -=2; 
    }
    if (seconds >= 1) {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
        seconds -=1; 
    }
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(75); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1.02*1023*1000L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void displayValues(double uv, double vcc, int raw) {
  display.clearDisplay();

  display.setTextColor(WHITE);

  display.setCursor(0, 0);

  display.setTextSize(1);
  display.print(F(" V "));
  display.print(vcc);

  display.print(F(" Raw "));
  display.print(raw);

  display.setCursor(0, 14);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.print(F("UV "));
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.print(uv);
  /* display.setTextSize(1);             // Normal 1:1 pixel scale */
  /* display.print(F("mW/cm2")); */

  display.display();
}
