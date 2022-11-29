/* This example shows how to change the range scaling factor
of the VL6180X. The sensor uses 1x scaling by default,
giving range measurements in units of mm. Increasing the
scaling to 2x or 3x makes it give raw values in units of 2
mm or 3 mm instead. In other words, a bigger scaling factor
increases the sensor's potential maximum range but reduces
its resolution. */

/* Note that for proper use, Prox1EN needs to be controlled.
 * Chip enable is pulled high on the VL6180 daughter board.
 */

#include <Wire.h>     // I2C library
#include <VL6180X.h>  // distance sensor library
#include <PCA95x5.h>  // I2C to GPIO library

// To try different scaling factors, change the following define.
// Valid scaling factors are 1, 2, or 3.
#define SCALING 3

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
}

unsigned long lastTime = 0;

void loop()
{
  unsigned long curTime = millis();
  unsigned long dt = curTime - lastTime;
  Serial.print("dt = ");
  Serial.print(dt);
  Serial.print(", ");
  //Serial.print("(Scaling = ");
  //Serial.print(sensor.getScaling());
  //Serial.print("x) ");

  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  lastTime = curTime;
}
