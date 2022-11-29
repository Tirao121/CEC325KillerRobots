/*
 * Filename:
 * Purpose:
 * 
 * Contributors:
 * Date:
 */

//Libraries
#include <SPI.h>
#include "WiFiNINA.h"  // for A4-A7 and wifi/bluetooth
#include <VL6180X.h>  // distance sensor library
#include <Adafruit_NeoPixel.h> // NeoPixel library
#include <PCA95x5.h>           // MUX library
#include <Servo.h>             // Servo
  // display libraries
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 display

//Pins & Defines
#define RIGHT_BUTTON_PIN   A0
#define BUZZ_PIN           2
#define SERVO_PIN          9
#define NEO_PIN           17  // WARNING! THIS IS GPIO NOT D NUMBER for NeoPixels
#define NEO_COUNT         18  // number of NeoPixels
#define NEO_MAG           100 // NeoPixel default magnitude
#define AIN1              0  // Motor driver A1
#define AIN2              1  // Motor driver A2
#define BIN1              8  // Motor driver B2
#define BIN2              A2  // Motor driver B1
#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  // display D/C

#define SCALING 3

//Objects
  // Declare our NeoPixel strip object:
  Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
  // objects for the VL6180X TOF distance sensor
  VL6180X sensor;      // distance sensor object
  PCA9535 muxU31;      // MUX object for U31 PCA9535
Servo myservo;
  // Display
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  GFXcanvas16 canvas(240,135);

//Variables
int mode = 0;
  //PID Constants
  float KP = 4;  // proportional control gain
  float KI = .1; // integral gain
  float KD = 8;    // derivative gain

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");
}

void loop() {
  //Declare variables

  //Switch between different modes
  switch(mode) {
    case 1:   //Standby
      //scan and detect
      break;
    case 2:   //Chase or Attack
      //PID
      //Alert function
      break;
    case 3:   //Only Alert
      //Alert only, arrived at obstacle or cannot reach
      break;
    case 4:   //Withdraw
      //After 5s or so, withdraw from target
      break;
    default:  //Idle (all operations suspended)
      //All operations off and safe
        /*LEDS
         * Light Sensor
         * Servos
         * tft
         * Buzzer
         */
      //Add message to tft saying idle
    break;
  }
}

void persue() {
  
}

// PID control
float pidControl(float error) {
  static float cumError = 0.0;
  static int lastError = 0;
  float d = KD*(error - lastError);
  cumError += KI*(float)error;
  Serial.print(cumError);
  Serial.print(",");
  Serial.print(d);
  Serial.print(",");
  lastError = error;
  return(KP*error + (float)cumError + d);
}
