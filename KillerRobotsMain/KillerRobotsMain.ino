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
#include <PeakDetection.h>
#include <Adafruit_NeoPixel.h> // NeoPixel library
#include <PCA95x5.h>           // MUX library
#include <Servo.h>             // Servo
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
PeakDetection peakDetection; // create PeakDetection object

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

  //Distance Sensor setup
 //I think we need this for the distance sensor -Jacob
  Wire.begin();
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P09, PCA95x5::Level::H);  // enable VL6180 distance sensor
  //I think we need this for the distance sensor -Jacob
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(100);

  //NeoPixel setup
  strip.begin();
  strip.clear();  // Set all pixel values to zero
  strip.show();   // Write values to the pixels

  pinMode(BUZZ_PIN, OUTPUT);    //Buzzer setup
  
  //Servo Setup
    //Distance sensor swivel
    myservo.attach(SERVO_PIN);
    myservo.write(90);
    //Wheel servos
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

  //Display setup (Must be after NeoPixels)
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, LOW); // disable TFT in case it was left running
  tft.init(135, 240);           // Init ST7789 240x135
  Serial.println("Display initialized");
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);

  //Peak detection setup
  peakDetection.begin(75, 4, 0.025); // sets the lag, threshold and influence
  peakDetection.setEpsilon(0.02);
}

//Global Variables
int proximity = 0;
int motorspeed = 50;
int angle = 90;       //swivel angle when change detected - default 90
double data = 0.0;

void loop() {
  //Declare variables
  
  //Switch between different modes
  switch(mode) {
    case 1:   //Standby
      angle = standby();
      break;
    case 2:   //Only Alert
      //Alert only, Use PID to turn to swivel angle, does not change modes until 
        //other robot detects change too
      break;
    case 3:   //Chase or Attack
      //PID
      //Alert function
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
      //Add message to tft saying idle?
    break;
  }
}

/* Detects change in radius, changes global var "mode" to alert if something detected
  and returns current angle swivel is in. */
double standby() {
  //Detect change in proximity
    //Use peak detect? Problem with swivel?
    //detects wheels at 155 and 1 (margin of 1 degree)
    //delay needed?
    //have detect peak for multiple values first?
  //Freeze wheels
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
  double pos = 0.0;
  for (pos = 1; pos <= 155; pos += .5) {
    int distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      data = (float)distance/765-1;  // 16-bit int -> +/- 1.0 range
    float stdpt = peakDetection.add(data); // adds a new data point
    int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    double filtered = peakDetection.getFilt(); // moving average
    //Only negative peak - enters field, not move away
    if(peak == -1) {
      mode = 2;
      return pos;
    } else {
      mode = 1;
    }
  }
  for (pos = 1; pos <= 155; pos -= .5) {
    int distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      data = (float)distance/765-1;  // 16-bit int -> +/- 1.0 range
    float stdpt = peakDetection.add(data); // adds a new data point
    int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    double filtered = peakDetection.getFilt(); // moving average
    //Only negative peak - enters field, not move away
    if(peak == -1) {
      mode = 2;
      return pos;
    } else {
      mode = 1;
    }
  }  
  
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

//attack function - must be called continuously in order to work
int alertToggle = 0;

int attack(int victimAngle) {
  if (IMU.accelerationAvailable()) {
    float x,y,z;
    IMU.readAcceleration(x, y, z);

  proximity = sensor.readRangeSingleMillimeters();
  float distanceError = proximity-proxThreshold;
  motorSpeed = pidControl(distanceError);

    int absSpeed = abs(motorSpeed);
  if(motorSpeed > 0) {
    analogWrite(AIN1, absSpeed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, absSpeed);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, absSpeed);
    analogWrite(BIN1, absSpeed);
    analogWrite(BIN2, 0);
  }
}

void Alert(){
  //Buzzer 
  if(alertToggle == 1){
    tone(BUZZ_PIN, 466.16, 150);
    alertToggle = 0;
  }
  else{
    tone(BUZZ_PIN, 200, 150);
    alertToggle = 1;
  }

  //Neopixels
  for(int ii = 0; ii < NEO_COUNT; ii++) {
    if(alertToggle == 1){
      strip.setPixelColor(ii, strip.Color(255,0,0));
      }
      else{
        strip.setPixelColor(ii, strip.Color(0,0,255));
    }
  }
  strip.show();

  //display
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.print(" _                   _");
  tft.println(" _( )                 ( )_");
  tft.println("(_, |      __ __      | ,_)");
  tft.println("   \'\    /  ^  \    /'/");
  tft.println("    '\'\,/\      \,/'/'");
  tft.println("      '\| []   [] |/'");
  tft.println("        (_  /^\  _)");
  tft.println("          \  ~  /");
  tft.println("          /HHHHH\\");
  tft.println("        /'/{^^^}\'\ ");
  tft.println("    _,/'/'  ^^^  '\'\,_");
  tft.println("   (_, |           | ,_)");
  tft.println("     (_)           (_)");

}

void withdraw() { 
  //waits for 5 seconds
  //slowly backs away while silencing buzzers, blink lights?
  //ends with turning off lights and returning to standby mode
  mode = 1;
}

void idle() {
  //lights
  //buzzer
  //tft
  //servos
  //flashing led
}
