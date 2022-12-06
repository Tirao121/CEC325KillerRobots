//Jacob Miller and Rita Sweep with Tft

#include <Wire.h>     // I2C library
#include <VL6180X.h>  // distance sensor library
#include <PCA95x5.h>  // I2C to GPIO library
#include <Servo.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 display

#define RIGHT_BUTTON_PIN   A0
#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  // display D/C
#define SD_CS             7

// Declare variables
// instatiate a tft object for the display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240,135);

#define SCALING 3

VL6180X sensor;      // distance sensor object
PCA9535 muxU31;      // MUX object for U31 PCA9535
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(115200);
  Wire.begin();
  myservo.attach(9);
  // I2C to GPIO chip setup
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P09, PCA95x5::Level::H);  // enable VL6180 distance sensor
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(100);
  myservo.write(pos);
  delay(500);

  // set pin directions
  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(TFT_CS, OUTPUT);

  digitalWrite(TFT_CS, LOW); // disable TFT in case it was left running
  
  // NeoPixel startup seems to interfere with display
  // so display needs to be started after NeoPixels
  // Setup TFT Display
  tft.init(135, 240);           // Init ST7789 240x135
  //tft.setSPISpeed(40000000);
  Serial.println("Display initialized");
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
}

int y = 0;

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 )degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    if (pos % 1 == 0) {
      y = sensor.readRangeSingleMillimeters();
      Serial.print(pos);
      Serial.print(", ");
      Serial.println(y);

      if (pos == 0 || pos == 180) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0,135);
      }

      int xplot = pos * 240/180;
      int yplot = 135 - (y * 135/765);

      tft.setCursor(xplot,yplot);
      tft.print(".");
    }
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    if (pos % 1 == 0) {
      y = sensor.readRangeSingleMillimeters();
      Serial.print(pos);
      Serial.print(", ");
      Serial.println(y);
            
      if (pos == 0 || pos == 180) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0,135);
      }

      int xplot = pos * 240/180;
      int yplot = 135 - (y * 135/765);

      tft.setCursor(xplot,yplot);
      tft.print(".");
    }
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
