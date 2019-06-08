#include "Screen.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
/* #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin) */
#define OLED_RESET     -1

char CATEGORY_NONE[] = "no UV";
char CATEGORY_LOW[] = "low";
char CATEGORY_MODERATE[] = "moderate";
char CATEGORY_HIGH[] = "high";
char CATEGORY_VERY_HIGH[] = "very high";
char CATEGORY_EXTREME[] = "extreme";

char *UV_INDEX_CATEGORY[] = {
    CATEGORY_NONE,
    CATEGORY_LOW,
    CATEGORY_LOW,
    CATEGORY_MODERATE,
    CATEGORY_MODERATE,
    CATEGORY_MODERATE,
    CATEGORY_HIGH,
    CATEGORY_HIGH,
    CATEGORY_VERY_HIGH,
    CATEGORY_VERY_HIGH,
    CATEGORY_VERY_HIGH,
    CATEGORY_EXTREME
};

Screen::Screen() {
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display->setRotation(2);
}

Screen::~Screen() {
}

void Screen::begin() {
    if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
    }
}

void Screen::on() {
  display->ssd1306_command(SSD1306_DISPLAYON);
}

void Screen::off() {
  display->ssd1306_command(SSD1306_DISPLAYOFF);
}

void Screen::show(int raw, double uv, double vcc) {
    display->clearDisplay();

    display->setTextColor(WHITE);

    display->setCursor(0, 0);

    display->setTextSize(1);
    display->print(F("Raw "));
    display->print(raw);

    display->print(F("  "));
    display->print(getUvCategory(getUvIndex(uv)));

    display->setCursor(0, 12);
    display->setTextSize(1);
    display->print(F("UV  "));
    display->setTextSize(2);
    display->print(uv);

    display->setCursor(87, 25);
    display->setTextSize(1);
    display->print(F("v"));
    display->print(vcc);

    display->display();
}

int Screen::getUvIndex(double uv) {
    return (byte)(uv + 0.5);
}

char* Screen::getUvCategory(int uvIndex) {
    if (uvIndex > 11) {
        return CATEGORY_EXTREME;
    }

    return UV_INDEX_CATEGORY[uvIndex];
}

