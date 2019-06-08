#ifndef Screen_h
#define Screen_h

#include "Arduino.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Screen {

    public: 

        Screen();

        ~Screen();

        void begin();

        void show(int raw, double uv, double vcc);

        void on();

        void off();

    private:

        Adafruit_SSD1306* display;

        int getUvIndex(double uv);

        char* getUvCategory(int uvIndex);

};
#endif
