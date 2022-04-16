/**************************************************************************
 
 Tire Simulator: Arduino-based 3-way temperature control for tire tread
                 => A test bench for RejsaRubberTrac tire temp sensors

  - SSD 1306 OLED display + rotary encoders to set temperatures
  - 3x DS18B20 temperature sensors for actual temperatures
  - PWM/MOS FET controlled silicone heating mats to heat up the tire (MOSFETs require a maximum PWM frequency of 20kHz)
  - PID controller to regulate the heaters to the set temperature (in software)

 **************************************************************************/

// Libraries for the SSD1306 OLED display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "fonts.h"
 #define _TASK_TIMECRITICAL
 #define _TASK_PRIORITY
#include <TaskScheduler.h>
// Libraries for the DS18B20 sensor
#include <OneWire.h>
#include <DallasTemperature.h>
// Library for the PID-based temperature controller
#include <PID_v1.h>
// Library for rotary encoder
#include <NewEncoder.h>

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


// DS18B20 sensor Pins and init
#define INNER_ONE_WIRE_BUS  32
#define MIDDLE_ONE_WIRE_BUS 33
#define OUTER_ONE_WIRE_BUS  25

OneWire innerOneWire(INNER_ONE_WIRE_BUS);
OneWire middleOneWire(MIDDLE_ONE_WIRE_BUS);
OneWire outerOneWire(OUTER_ONE_WIRE_BUS);
DallasTemperature innerSensor(&innerOneWire);
DallasTemperature middleSensor(&middleOneWire);
DallasTemperature outerSensor(&outerOneWire);


// PID initialization
#define PWM_RESOLUTION 255
#define INNER_PWM     4 // do not change to pins other than D9 or D10 (on an Arduino Nano)
//#define MIDDLE_PWM  ? // Arduino Nano only has two fast PWM output pins, so we simplify the setup and go for only two heating mats for inner and outer
#define OUTER_PWM    2 // do not change to pins other than D9 or D10 (on an Arduino Nano)

// variable declarations
double innerSetpoint, middleSetpoint = 25, outerSetpoint = 15;
double innerActual, middleActual, outerActual;
double innerPWMOutput, middlePWMOutput, outerPWMOutput;

//Specify the links and initial tuning parameters
double Kp=5, Ki=3, Kd=3;
PID innerPID(&innerActual, &innerPWMOutput, &innerSetpoint, Kp, Ki, Kd, DIRECT);
PID middlePID(&middleActual, &middlePWMOutput, &middleSetpoint, Kp, Ki, Kd, DIRECT);
PID outerPID(&outerActual, &outerPWMOutput, &outerSetpoint, Kp, Ki, Kd, DIRECT);


NewEncoder innerEncoder(26, 27, -20, 200, 99, FULL_PULSE); // temperature range: -20 to 200 degrees celsius
//NewEncoder outerEncoder(26, 27, -20, 200, 0, FULL_PULSE); // temperature range: -20 to 200 degrees celsius
void ESP_ISR innerEncoderCallback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
//void ESP_ISR outerEncoderCallback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
volatile NewEncoder::EncoderState newInnerEncoderState, newOuterEncoderState;
volatile bool newInnerEncoderValue = false, newOuterEncoderValue = false;

// Function declarations
void updateSetpoints();
void updateDisplay();
void updateInnerSensor();
void updateMiddleSensor();
void updateOuterSensor();
void updateInnerPWM();
void updateMiddlePWM();
void updateOuterPWM();

void drawCenteredTemp(int16_t, int16_t, int16_t, boolean);
void drawCenteredString(int16_t, int16_t, const char*, boolean);
void drawCenteredProgressbar(int16_t, int16_t, int16_t, int16_t, uint16_t);
/*
void analogWriteScaled_Init(void);
void analogWriteScaledD9(uint8_t);
void analogWriteScaledD10(uint8_t);
*/


Scheduler r, hpr;

//Tasks
Task tUpdateSetpoints(200, TASK_FOREVER, &updateSetpoints, &hpr);
Task tUpdateDisplay(200, TASK_FOREVER, &updateDisplay, &hpr);

Task tUpdateInner(1000, TASK_FOREVER, &updateInnerSensor, &r);
Task tUpdateMiddle(1000, TASK_FOREVER, &updateMiddleSensor, &r);
Task tUpdateOuter(1000, TASK_FOREVER, &updateOuterSensor, &r);


void setup() {
    Serial.begin(115200);
    Wire.begin(GPIOSDA,GPIOSCL); // initialize I2C channel w/ I2C pins from platformio.ini
    NewEncoder::EncoderState innerEncoderState;

    if (!innerEncoder.begin()) {
        Serial.println("Encoder Failed to Start. Check pin assignments and available interrupts. Aborting.");
        while (1) {
        yield();
        }
    } else {
        innerEncoder.getState(innerEncoderState);
        Serial.print("Encoder Successfully Started at value = ");
        innerSetpoint = innerEncoderState.currentValue;
        Serial.println(innerSetpoint);
        innerEncoder.attachCallback(innerEncoderCallback);
    }
  
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    // Clear the buffer; initialize
    display.setTextWrap(false);
    display.setTextSize(1);
//    display.setFont(&Lato_Hairline_16);

    // init PWM output scaling to 20kHz
    //analogWriteScaled_Init();
    //2do: set pinMode for PWM Mosfet...

    // set PID ranges and activate
    innerPID.SetOutputLimits(0, PWM_RESOLUTION);
    middlePID.SetOutputLimits(0, PWM_RESOLUTION);
    outerPID.SetOutputLimits(0, PWM_RESOLUTION);

    innerPID.SetMode(AUTOMATIC);
    middlePID.SetMode(AUTOMATIC);
    outerPID.SetMode(AUTOMATIC);

    // start TaskScheduler runner
    r.setHighPriorityScheduler(&hpr); // high prio for display and encoders
    r.enableAll(true); // this will recursively enable the higher priority tasks as well
}

void loop() {
    r.execute();
}

void updateSetpoints() {
    // read new setpoint from encoder
    int16_t currentValue;
    NewEncoder::EncoderClick currentClick;

    if (newInnerEncoderValue) {
        noInterrupts();
        currentValue = newInnerEncoderState.currentValue;
        currentClick = newInnerEncoderState.currentClick;
        newInnerEncoderValue = false;
        interrupts();
        Serial.print("Encoder: ");
        if (currentValue != innerSetpoint) {
        Serial.println(currentValue);
        innerSetpoint = currentValue;
        } else {
            switch (currentClick) {
                case NewEncoder::UpClick:
                Serial.println("at upper limit.");
                break;

                case NewEncoder::DownClick:
                Serial.println("at lower limit.");
                break;

                default:
                break;
            }
        }
    }
}

void updateDisplay() {
    // refresh screen
    display.clearDisplay();

    drawCenteredString(21, 5, "INNER", false);
    drawCenteredTemp(21, 25, innerSetpoint, true);
    drawCenteredTemp(21, 45, innerActual, false);
    drawCenteredProgressbar(21, 55, 25, 5, map(innerPWMOutput, 0, PWM_RESOLUTION, 0, 100));

    drawCenteredString(42+21, 5, "MIDDLE", false);
    drawCenteredTemp(42+21, 25, middleSetpoint, true);
    drawCenteredTemp(42+21, 45, middleActual, false);
    drawCenteredProgressbar(42+21, 55, 25, 5, map(middlePWMOutput, 0, PWM_RESOLUTION, 0, 100));

    drawCenteredString(84+21, 5, "OUTER", false);
    drawCenteredTemp(84+21, 25, outerSetpoint, true);
    drawCenteredTemp(84+21, 45, outerActual, false);
    drawCenteredProgressbar(84+20, 55, 25, 5, map(outerPWMOutput, 0, PWM_RESOLUTION, 0, 100));

    display.display();
}

void updateInnerSensor() {
    // get temperature readings
    innerSensor.requestTemperatures();
    innerActual = innerSensor.getTempCByIndex(0);

    tUpdateInner.setCallback(&updateInnerPWM);
    tUpdateInner.forceNextIteration();
}

void updateMiddleSensor() {
    // get temperature readings
    middleSensor.requestTemperatures();
    middleActual = middleSensor.getTempCByIndex(0);

    tUpdateMiddle.setCallback(&updateMiddlePWM);
    tUpdateMiddle.forceNextIteration();
}

void updateOuterSensor() {
    // get temperature readings
    outerSensor.requestTemperatures();
    outerActual = outerSensor.getTempCByIndex(0);

    tUpdateOuter.setCallback(&updateOuterPWM);
    tUpdateOuter.forceNextIteration();
}

void updateInnerPWM() {
    // update PID calculations
    innerPID.Compute();

    // update PWM output frequencies
//    analogWriteScaledD9(innerPWMOutput);
    // middlePWMOutput is currently not connected due to only 2x heating mats connected and only 2x PWM output Pins on the Arduino Nano
    // middlePID is displayed only for convenience reasons
//    analogWriteScaledD10(outerPWMOutput);

    tUpdateInner.setCallback(&updateInnerSensor);
}

void updateMiddlePWM() {
    // update PID calculations
    middlePID.Compute();

    //2do PWM

    tUpdateMiddle.setCallback(&updateMiddleSensor);
}

void updateOuterPWM() {
    // update PID calculations
    outerPID.Compute();

    //2do PWM

    tUpdateOuter.setCallback(&updateOuterSensor);
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

void ESP_ISR innerEncoderCallback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr) {
    (void) encPtr;
    (void) uPtr;
    memcpy((void *)&newInnerEncoderState, (void *)state, sizeof(NewEncoder::EncoderState));
    newInnerEncoderValue = true;
    
    tUpdateSetpoints.forceNextIteration();
}

/*
// Scale PWM output to 20kHz
// Kudos to Coding_Badly (https://forum.arduino.cc/t/how-to-get-20khz-pwm-on-pin-d9/132857/15)
void analogWriteScaled_Init(void)
{
  // Stop the timer while we muck with it

  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
  
  // Set the timer to mode 14...
  //
  // Mode  WGM13  WGM12  WGM11  WGM10  Timer/Counter Mode of Operation  TOP   Update of OCR1x at TOV1  Flag Set on
  //              CTC1   PWM11  PWM10
  // ----  -----  -----  -----  -----  -------------------------------  ----  -----------------------  -----------
  // 14    1      1      1      0      Fast PWM                         ICR1  BOTTOM                   TOP
  
  // Set output on Channel A and B to...
  //
  // COM1z1  COM1z0  Description
  // ------  ------  -----------------------------------------------------------
  // 1       0       Clear OC1A/OC1B on Compare Match (Set output to low level).
  
  TCCR1A = 
      (1 << COM1A1) | (0 << COM1A0) |   // COM1A1, COM1A0 = 1, 0
      (1 << COM1B1) | (0 << COM1B0) |
      (1 << WGM11) | (0 << WGM10);      // WGM11, WGM10 = 1, 0

  // Set TOP to...
  //
  // fclk_I/O = 16000000
  // N        = 1
  // TOP      = 799
  //
  // fOCnxPWM = fclk_I/O / (N * (1 + TOP))
  // fOCnxPWM = 16000000 / (1 * (1 + 799))
  // fOCnxPWM = 16000000 / 800
  // fOCnxPWM = 20000

  ICR1 = 799;

  // Ensure the first slope is complete

  TCNT1 = 0;

  // Ensure Channel A and B start at zero / off

  OCR1A = 0;
  OCR1B = 0;

  // We don't need no stinkin interrupts

  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

  // Ensure the Channel A and B pins are configured for output
  DDRB |= (1 << DDB1);
  DDRB |= (1 << DDB2);

  // Start the timer...
  //
  // CS12  CS11  CS10  Description
  // ----  ----  ----  ------------------------
  // 0     0     1     clkI/O/1 (No prescaling)

  TCCR1B =
      (0 << ICNC1) | (0 << ICES1) |
      (1 << WGM13) | (1 << WGM12) |              // WGM13, WGM12 = 1, 1
      (0 << CS12) | (0 << CS11) | (1 << CS10);
}

// Kudos to Mat 13 (https://forum.arduino.cc/t/how-to-get-20khz-pwm-on-pin-d9/132857/17)
void analogWriteScaledD9(uint8_t value)
{
    // input variable value varies from 0 to PWM_RESOLUTION (default: 255)
    // but awaited range in OCR1A/OCR1B is from 0 to 799 and nothing else!
    // using 780 as max scaling range to account for some inaccuracies
    OCR1A = constrain(map(value, 0, PWM_RESOLUTION, 0, 780), 0, 799);
}

void analogWriteScaledD10(uint8_t value)
{
    // input variable value varies from 0 to PWM_RESOLUTION (default: 255)
    // but awaited range in OCR1A/OCR1B is from 0 to 799 and nothing else!
    // using 780 as max scaling range to account for some inaccuracies
    OCR1B = constrain(map(value, 0, PWM_RESOLUTION, 0, 780), 0, 799);
}
*/