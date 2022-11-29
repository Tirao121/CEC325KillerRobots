/*
  LED/Button Central interface

  Connects to a peripheral that has LED and button
  characteristics. Controls the LED with its button
  and turns on its own LED and buzzer when the 
  peripheral pushes its button.

  Note that using an SPI device (display/SD card)
  interferes with the LED_BUILTIN. If you put a
  pinMode statement right before you set the LED
  state it will often still work as long as the
  SPI isn't used during the period of interest.

  Seth McNeill
  2022 November 21


  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_LED                 "19B10001-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_BUTTON              "19B10012-E8F2-537E-4F6C-D104768A1214"

// variables for button
#define RIGHT_BUTTON_PIN   A0
#define BUZZ_PIN           2
const int buttonPin = RIGHT_BUTTON_PIN;
const int ledPin = LED_BUILTIN;
int oldButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(3000);
//  while (!Serial);

  // configure the button pin as input
  pinMode(buttonPin, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

  // start scanning for peripherals
  BLE.scanForUuid(BLE_UUID_PERIPHERAL);
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "BUTTON_LED") {
      Serial.print("Wrong local name: ");
      Serial.println(peripheral.localName());
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid(BLE_UUID_PERIPHERAL);
  }
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic(BLE_UUID_LED);
  // retrieve the Button characteristic
  BLECharacteristic buttonCharacteristic = peripheral.characteristic(BLE_UUID_BUTTON);

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  } else {
    Serial.println("Connected to LED characteristic");
  }

  if (!buttonCharacteristic) {
    Serial.println("Peripheral does not have button characteristic!");
    peripheral.disconnect();
    return;
  } else if (!buttonCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable button characteristic!");
    peripheral.disconnect();
    return;
  } else if (!buttonCharacteristic.canSubscribe()) {
    Serial.println("Peripheral does not allow button subscriptions (notify)");
  } else if(!buttonCharacteristic.subscribe()) {
    Serial.println("Subscription failed!");
  } else {
    Serial.println("Connected to button characteristic");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // read the button pin
    int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (!buttonState) {
        Serial.println("button pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeValue((byte)0x01);
      } else {
        Serial.println("button released");

        // button is released, write 0x00 to turn the LED off
        ledCharacteristic.writeValue((byte)0x00);
      }
    }
        if(buttonCharacteristic && buttonCharacteristic.canRead() && buttonCharacteristic.valueUpdated()) {
      byte peripheralButtonState;
      buttonCharacteristic.readValue(peripheralButtonState);
      if(!peripheralButtonState) {
        digitalWrite(ledPin, HIGH);
        tone(BUZZ_PIN, 1000);
      } else {
        digitalWrite(ledPin, LOW);
        noTone(BUZZ_PIN);
      }
      Serial.print("peripheralButtonState = ");
      Serial.println(peripheralButtonState);
    }

  }

  Serial.println("Peripheral disconnected");
}
