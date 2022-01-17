/**************************************************************************
 
 Tire Simulator: Arduino-based 3-way temperature control for tire tread
                 => A test bench for RejsaRubberTrac tire temp sensors

  - SSD 1306 OLED display + rotary encoders to set temperatures
  - 3x DS18B20 temperature sensors for actual temperatures
  - PWM/MOS FET controlled silicone heating mats to heat up the tire
  - PID controller to regulate the heaters to the set temperature (in software)

 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "fonts.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for Adafruit 128x64, 0x3C for Adafruit 128x32, 0x3C for China 128x64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Function declarations
void drawCenteredTemp(int16_t, int16_t, int16_t, boolean);
void drawCenteredString(int16_t, int16_t, const char*, boolean);
void drawCenteredProgressbar(int16_t, int16_t, int16_t, int16_t, uint16_t);


void setup() {
    Serial.begin(9600);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    // Clear the buffer; initialize
    display.clearDisplay();
    display.setTextWrap(false);
    display.setTextSize(1);
//    display.setFont(&Lato_Hairline_16);

    drawCenteredString(21, 5, "INNER", false);
    drawCenteredTemp(21, 25, 100, true);
    drawCenteredTemp(21, 45, 69, false);
    drawCenteredProgressbar(21, 55, 25, 5, 100);

    drawCenteredString(42+21, 5, "MIDDLE", false);
    drawCenteredTemp(42+21, 25, 100, true);
    drawCenteredTemp(42+21, 45, 78, false);
    drawCenteredProgressbar(42+21, 55, 25, 5, 80);

    drawCenteredString(84+21, 5, "OUTER", false);
    drawCenteredTemp(84+21, 25, 90, true);
    drawCenteredTemp(84+21, 45, 64, false);
    drawCenteredProgressbar(84+20, 55, 25, 5, 40);

    display.display();
}

void loop() {
}

void drawCenteredProgressbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t percentage)
{
   percentage = percentage > 100 ? 100 : percentage; // normalize value to 100
   float barWidth = ((float)(w-4) / 100) * percentage;
   display.drawRect((x-w/2), y, w, h, SSD1306_WHITE);
   display.fillRect((x-w/2)+2, y+2, barWidth, h-4, SSD1306_WHITE);
}

void drawCenteredTemp(int16_t x, int16_t y, int16_t temp, boolean inverted)
{
    char cstr[5 + sizeof(char)];
    //sprintf(cstr, "%d\xF8""C", temp); // sprintf with degrees celsius
    //sprintf(cstr, "%d\xB0""C", temp); // sprintf with degrees celsius
    sprintf(cstr, "%d%cC", temp, 248); // sprintf with degrees celsius
    //sprintf(cstr, "%d%cC", temp, 126); // sprintf with degrees celsius (hacked as #126 in the modified font)
    drawCenteredString(x, y, cstr, inverted);
}

void drawCenteredString(int16_t x, int16_t y, const char *buf, boolean inverted)
{
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, x, y, &x1, &y1, &w, &h);
    
    if (inverted) {
        display.fillRect((x-w/2)-2, y1-2, w+4, h+4, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor((x-w/2), y);
    display.print(buf);
}
