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
// use the first ESP32 channels of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13
#define PWM_RESOLUTION 255

// use 20 kHz as a LEDC base frequency (= max. PWM frequency)
#define LEDC_BASE_FREQ 20000

#define INNER_PWM     4
//#define MIDDLE_PWM  ? // currently: simplified setup with only two heating mats for inner and outer
#define OUTER_PWM     2

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

void ESP_ISR innerEncoderCallback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
//void ESP_ISR outerEncoderCallback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = PWM_RESOLUTION);


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

    // set PID ranges and activate
    innerPID.SetOutputLimits(0, PWM_RESOLUTION);
    middlePID.SetOutputLimits(0, PWM_RESOLUTION);
    outerPID.SetOutputLimits(0, PWM_RESOLUTION);

    innerPID.SetMode(AUTOMATIC);
    middlePID.SetMode(AUTOMATIC);
    outerPID.SetMode(AUTOMATIC);

    // init PWM output scaling to 20kHz
    // Setup PWM timer and attach timer to a led pin
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(INNER_PWM, LEDC_CHANNEL_0);
//    ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
//    ledcAttachPin(MIDDLE_PWM, LEDC_CHANNEL_1);
    ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(OUTER_PWM, LEDC_CHANNEL_2);

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

    // update PWM output frequency
    ledcAnalogWrite(LEDC_CHANNEL_0, innerPWMOutput);

    tUpdateInner.setCallback(&updateInnerSensor);
}

void updateMiddlePWM() {
    // update PID calculations
    middlePID.Compute();

    // update PWM output frequency
//    ledcAnalogWrite(LEDC_CHANNEL_1, middlePWMOutput);
// middlePWMOutput is currently not connected due to only 2x heating mats connected and only 2x PWM output Pins on the Arduino Nano
// middlePID is displayed only for convenience reasons

    tUpdateMiddle.setCallback(&updateMiddleSensor);
}

void updateOuterPWM() {
    // update PID calculations
    outerPID.Compute();

    // update PWM output frequency
    ledcAnalogWrite(LEDC_CHANNEL_2, outerPWMOutput);

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

// Arduino like analogWrite for ESP32 LEDC
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax) {
    // calculate duty, 8191 from 2 ^ 13 - 1
    uint32_t duty = (8191 / valueMax) * min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
}