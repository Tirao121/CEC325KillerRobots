/* peakDetectDemo.ino
 *  
 *  Demo of peak detection using the CEC325 v0.3 board.
 *  
 *  Seth McNeill
 *  2022 March 25
 *  Modified by Rita and Ivan
 */
#include <Wire.h>     // I2C library
#include <VL6180X.h>  // distance sensor library
#include <PeakDetection.h> // import lib
#include "Adafruit_SHT31.h"   // Temperature and Humidity sensor
#include <PCA95x5.h>
#include <WiFiNINA.h>  // for RGB LED
#include <Adafruit_NeoPixel.h> // NeoPixel library
#include<Anitracks_ADS7142.h>  // ADC library
#include <Servo.h>             // Servo


Servo myservo;

#define SCALING 3
#define RIGHT_BUTTON_PIN  A7 // Doesn't work yet. Check whether WiFiNINA can pullup internally
#define BUZZ_PIN          A2
#define SERVO_PIN          9

// pin definitions:
#define NEO_PIN           17  // WARNING! THIS IS GPIO NOT D NUMBER for NeoPixels
#define NEO_COUNT         18  // number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

VL6180X sensor;      // distance sensor object
PeakDetection peakDetection; // create PeakDetection object
PCA9535 muxU31;      // MUX object for U31 PCA9535

void setup() {
  Serial.begin(115200); // set the data rate for the Serial communication
  while(!Serial);
  //delay(2000);
  // I2C to GPIO chip setup
  Wire.begin();
  
  // Setup NeoPixels
  strip.begin();
  strip.clear();  // Set all pixel values to zero
  strip.show();   // Write values to the pixels

  // Distance sensor setup
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(100);

  //Distance sensor swivel
    myservo.attach(SERVO_PIN);
    myservo.write(90);
  
  peakDetection.begin(75, 4, 0.025); // sets the lag, threshold and influence
  peakDetection.setEpsilon(0.02);
  
  Serial.println("data,peak,filt,std");
}

void loop() {
    int distance = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    double data = (float)distance/765-1;  // 16-bit int -> +/- 1.0 range

    float stdpt = peakDetection.add(data); // adds a new data point
    int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    double filtered = peakDetection.getFilt(); // moving average
    Serial.print(data); // print data
    Serial.print(",");
    Serial.print(peak); // print peak status
    Serial.print(",");
    Serial.print(filtered); // print moving average
    Serial.print(",");
    Serial.println(stdpt); // print std dev

     switch (peak)
       {
       case 1:
          for(int ii = 0; ii < NEO_COUNT; ii++) {
              strip.setPixelColor(ii, strip.Color(0,0,255));
              }
              strip.show();
          break;
       case -1:
          for(int ii = 0; ii < NEO_COUNT; ii++) {
              strip.setPixelColor(ii, strip.Color(255,0,0));
              }
              strip.show();
          break;
       
       default:
          for(int ii = 0; ii < NEO_COUNT; ii++) {
              strip.setPixelColor(ii, strip.Color(0,255,0));
              }
              strip.show();
          break;
       }

    delay(50);
    
}
