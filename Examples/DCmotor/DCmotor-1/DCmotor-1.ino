// Jacob Block and Ivan Filuk

#include <Wire.h>     // I2C library
#include <VL6180X.h>  // distance sensor library
#include <PCA95x5.h>  // I2C to GPIO library
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h>

#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  
#define AIN1              0 //L forward
#define AIN2              1 //L back

#define BIN1              8 // R back
#define BIN2              A2 //R forward

// To try different scaling factors, change the following define.
// Valid scaling factors are 1, 2, or 3.
#define SCALING 3


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);
VL6180X sensor;      // distance sensor object
PCA9535 muxU31;      // MUX object for U31 PCA9535

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // I2C to GPIO chip setup
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P09, PCA95x5::Level::H);  // enable VL6180 distance sensor

  // Distance sensor setup
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(100);

    digitalWrite(TFT_CS, LOW); // disable TFT in case it was left running
  // Setup TFT Display
  tft.init(135, 240);           // Init ST7789 240x135
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);
}




void loop()
{

  analogWrite(BIN2, 255); //R forward
  analogWrite(AIN1, 0); //L forward
  analogWrite(AIN2, 255); //L back
  analogWrite(BIN1, 0); //R back
  delay(750);
  //delay(4000); //angle multiplied by 76/9
  analogWrite(BIN2, 0); //R forward
  analogWrite(AIN1, 0); //L forward
  analogWrite(AIN2, 0); //L back
  analogWrite(BIN1, 0); //R back
  delay(1000);
  analogWrite(BIN2, 0); //R forward
  analogWrite(AIN1, 255); //L forward
  analogWrite(AIN2, 0); //L back
  analogWrite(BIN1, 255); //R back
  delay(750);

  }
 
  



