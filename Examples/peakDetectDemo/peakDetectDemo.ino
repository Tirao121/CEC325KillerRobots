/* peakDetectDemo.ino
 *  
 *  Demo of peak detection using the CEC325 v0.3 board.
 *  
 *  Seth McNeill
 *  2022 March 25
 *  Modified for v0.5 board 2022 October 27
 */

#include <PeakDetection.h> // import lib
#include "Adafruit_SHT31.h"   // Temperature and Humidity sensor
#include <PCA95x5.h>  // I2C to GPIO library
#include<Anitracks_ADS7142.h>  // ADC library


#define SHT31_ADDR        0x44  // default, 0x45 is alternate

#define RIGHT_BUTTON_PIN  A7 // Doesn't work yet. Check whether WiFiNINA can pullup internally
#define BUZZ_PIN          A2
#define ADC1_ADDR 0x18  // ADS7142 I2C address, temp0 and pot
#define ADC2_ADDR 0x1F  // ADS7142 I2C address, light1 and light2

ADS7142 adc1(ADC1_ADDR); // Create ADS7142 object
ADS7142 adc2(ADC2_ADDR); // Create ADS7142 object

PeakDetection peakDetection; // create PeakDetection object
Adafruit_SHT31 sht31 = Adafruit_SHT31();
PCA9535 muxU31;      // MUX object for U31 PCA9535

void setup() {
  Serial.begin(115200); // set the data rate for the Serial communication
  while(!Serial);
  //delay(2000);
  // I2C to GPIO chip setup
  Wire.begin();
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P08, PCA95x5::Level::L);  // disable SHT31 temp/humidity
  delay(100);
  muxU31.write(PCA95x5::Port::P08, PCA95x5::Level::H);  // enable SHT31 temp/humidity
  delay(100);

  peakDetection.begin(75, 4, 0.001); // sets the lag, threshold and influence
  peakDetection.setEpsilon(0.02);

  if (! sht31.begin(SHT31_ADDR)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Failed to start SHT31");
  } else {
    // turn off the heater (increases temp reading by about 3C)
    sht31.heater(false);
  }
  if(!adc1.begin()) {
    Serial.println("ADC1 failed to begin");
  }
  if(!adc2.begin()) {
    Serial.println("ADC2 failed to begin");
  }
  //Serial.print("EPSILON = ");
  //Serial.println(peakDetection.getEpsilon());
  Serial.println("data,peak,filt,std");
}

void loop() {
    uint16_t temp0;
    uint16_t pot;
    uint16_t light1;
    uint16_t light2;
    if(!adc1.read2Ch(&temp0, &pot)) {
      Serial.println("adc1 failed");
    }
    if(!adc2.read2Ch(&light1, &light2)) {
      Serial.println("adc2 failed");
    }
    double data = (double)analogRead(A0)/512-1; // reads the value of the sensor and converts to a range between -1 and 1
    float tC = sht31.readTemperature();
    float humidity = sht31.readHumidity();
    float tF = tC*9/5+32;
    //data = humidity/50 - 1;
    data = (double)pot/32768-1;  // 16-bit int -> +/- 1.0 range

    float stdpt = peakDetection.add(data); // adds a new data point
//    int peak = peakDetection.getPeak(); // returns 0, 1 or -1
    int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    double filtered = peakDetection.getFilt(); // moving average
    Serial.print(data); // print data
    Serial.print(",");
    Serial.print(peak); // print peak status
    Serial.print(",");
    Serial.print(filtered); // print moving average
    Serial.print(",");
    Serial.println(stdpt); // print std dev
    delay(50);
}
