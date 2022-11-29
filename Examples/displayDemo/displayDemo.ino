/* displayDemo.ino
 * 
 * Demonstrates the Adafruit 1.14" 240x135 Color TFT display
 * 
 * Seth McNeill
 * 2022 September 26
 */

// Include libraries
// display libraries
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 display
#include <Fonts/FreeSansBold9pt7b.h>

// Define pins
#define RIGHT_BUTTON_PIN   A0
#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  // display D/C
#define SD_CS             7

// Declare variables
// instatiate a tft object for the display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240,135);

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");

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
  tft.setTextWrap(true);
  tft.setTextSize(3);
  tft.println("CEC 326 v0.5");
  tft.setTextColor(ST77XX_RED);
  tft.println("Welcome!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.println("Yellow");

  //delay(1000);
}

void loop() {
  rButtonWait();
  // clear the screen
  tft.fillScreen(ST77XX_BLACK);
  // Rotation demo
  tft.setTextColor(ST77XX_WHITE);
  for(int ii = 0; ii < 4; ii++) {
    tft.setRotation(ii);
    tft.setCursor(0, 0);
    tft.print("R: ");
    tft.println(ii);
    delay(2000);
  }

  rButtonWait();
  // draw single character
  tft.fillScreen(ST77XX_BLACK);
  // Rotation demo
  tft.setTextColor(ST77XX_RED);
  tft.setRotation(1);
  uint16_t screenWidth = tft.width();
  uint16_t screenHeight = tft.height();
  for(int ii = 0; ii < screenWidth+1; ii++) {
    tft.fillScreen(ST77XX_BLACK);
    tft.drawChar(ii, screenHeight/2-8, 0x7F, ST77XX_RED, ST77XX_WHITE, 3);
    //delay(75);
  }
  // Notice LED_BUILTIN flickering with SPI access!
  // Also, notice the screen flickering

  rButtonWait();
  // flicker free drawing
  for(int ii = 0; ii < screenWidth+1; ii++) {
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(ii, 20);
    canvas.drawChar(ii, screenHeight/2-8, 0x7F, ST77XX_RED, ST77XX_WHITE, 3);
    canvas.drawLine(ii, screenHeight/2-8, 0,0, ST77XX_BLUE);
    canvas.setTextSize(3);
    canvas.println("Hello world!");
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  }

  rButtonWait();
  // no text wrap gives scrolling text
  canvas.setTextWrap(false);
  for(int ii = 0; ii < screenWidth+1; ii += 1) {
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(ii, screenHeight/2-8);
    canvas.drawLine(ii, screenHeight/2-8, 0,0, ST77XX_RED);
    canvas.setTextSize(3);
    canvas.println("Hello world!");
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  }

  // new fonts
  rButtonWait();
  tft.setFont(&FreeSansBold9pt7b);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,40);
  tft.println("New \nFont");
  tft.setFont();  // return to regular font
  delay(1000);
}

bool rButtonWait() {
  while(digitalRead(RIGHT_BUTTON_PIN)) {
    delay(10);
  }
  return(true);
}
