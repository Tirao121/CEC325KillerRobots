/* PID_Ramp.ino
 *  
 * Uses PID Controller to balance on Ramp
 * 
 * Dominic Stringer and Jacob Block
 * 2022 November 15
 */

#include <Adafruit_NeoPixel.h> // NeoPixel library
#include <PCA95x5.h>           // MUX library
#include <Servo.h>             // Servo
#include "Anitracks_ADS7142.h"  // ADC library
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 display
#include <Arduino_LSM6DSOX.h>

#define RIGHT_BUTTON_PIN   A0
#define BUZZ_PIN           2
#define SERVO_PIN          9
#define NEO_PIN           17  // WARNING! THIS IS GPIO NOT D NUMBER for NeoPixels
#define NEO_COUNT         18  // number of NeoPixels
#define NEO_MAG           100 // NeoPixel default magnitude
#define AIN1              0  // Motor driver A
#define AIN2              1  // Motor driver A
#define BIN1              8  // Motor driver A
#define BIN2              A2  // Motor driver A
#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  // display D/C
#define SD_CS             7

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
// objects for the VL6180X TOF distance sensor

PCA9535 muxU31;      // MUX object for U31 PCA9535
Servo myservo;
// instatiate a tft object for the display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240,135);
ADS7142 adc2(0x18);  // POT and TEMP

enum States {pid, halt};
States state = halt;

float KP = 400  ;  // proportional control gain
float KI = 10; // integral gain
float KD = 30;    // derivative gain

// PID control
int pidControl(float error) {
  static float cumError = 0.0;
  static int lastError = 0;
  int d = KD*(error - lastError);
  cumError += KI*(float)error;
  Serial.print(cumError);
  Serial.print(",");
  Serial.print(d);
  Serial.print(",");
  lastError = error;
  return(KP*error + (int)cumError + d);
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  // start NeoPixels
  strip.begin();

  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object

  Wire.begin();
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P09, PCA95x5::Level::H);  // enable VL6180 distance sensor

  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(TFT_CS, OUTPUT);

  // NeoPixel startup seems to interfere with display
  // so display needs to be started after NeoPixels
  // Setup TFT Display
  tft.init(135, 240);           // Init ST7789 240x135
  //tft.setSPISpeed(120000000);
  //Serial.println("Display initialized");
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextWrap(true);
  tft.setTextSize(3);
  tft.println("PID Distance");
  tft.setTextColor(ST77XX_RED);
  tft.println("Welcome!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.println("Yellow");

  myservo.write(115);
  adc2.begin();
  strip.clear();
  strip.fill(strip.Color(0,NEO_MAG,0),0,5);
  strip.show();  
  delay(1000);
}

// distance variables
float proxThreshold = 0;
int leftButtonState;
int leftLastButtonState = 1;
int rightButtonState;
int rightLastButtonState = 1;
int motorSpeed = 50;

void loop() {
  uint16_t ch0a;
  uint16_t ch1a;
  rightButtonState = digitalRead(RIGHT_BUTTON_PIN);
  if(!adc2.read2Ch(&ch0a, &ch1a)) {
    Serial.println("read2CH failed");
  }
  proxThreshold = map(ch1a, 0, 65536, 0, 765)/765.0;

  if(rightButtonState != rightLastButtonState && !rightButtonState) {
    if(state == halt) {
      strip.fill(strip.Color(NEO_MAG,0,0),0,5);
      strip.show();
      state = pid;
    } else if(state == pid) {
      strip.fill(strip.Color(0,NEO_MAG,0),0,5);
      strip.show();
      state = halt;
    }
  }

  String dispStr = "PID Distance Control\n";

  if (IMU.accelerationAvailable()) {
    float x,y,z;
    IMU.readAcceleration(x, y, z);
    serial.println(x " ," y " ," z);

    dispStr += "Angle Reading: ";
    dispStr += y;
    dispStr += "\n";
  
    dispStr += "Target: ";
    dispStr += proxThreshold;
    dispStr += "\n";
  
    // setpoint is proxThreshold
    // plant variable is proximity
    // control variable is motorSpeed  
    
    // control logic
    float error = -(y+proxThreshold);
    dispStr += "Error: ";
    dispStr += error;
    dispStr += "\n";
    Serial.print(error);
    Serial.print(",");
    switch(state) {
      case pid: motorSpeed = pidControl(error); dispStr += "PID\n"; break;
      case halt: motorSpeed = 0; dispStr += "HALT\n"; Serial.print("0,0,"); break;
      default: motorSpeed = 0; state = halt; Serial.println("State corrupt"); dispStr += "State currupt";
    }
  
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
  
    Serial.print("\n");
  
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0,0);
    canvas.setTextSize(2);
    canvas.println(dispStr);
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  }
//  leftLastButtonState = leftButtonState;
  rightLastButtonState = rightButtonState;
}
