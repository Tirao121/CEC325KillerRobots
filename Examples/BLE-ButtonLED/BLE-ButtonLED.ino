/*
  BLE-ButtonLED

  This sets up an output (read/write) LED and an input (READ/NOTIFY)
  on an Nano Connect RP2040 based board.

  Seth McNeill
  2022 November 21
  Based on the ArduinoBLE LED sketch combined with ButtonLED

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_LED                 "19B10001-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_BUTTON              "19B10012-E8F2-537E-4F6C-D104768A1214"

BLEService ledService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic LEDCharacteristic(BLE_UUID_LED, BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEByteCharacteristic buttonCharacteristic(BLE_UUID_BUTTON, BLERead | BLENotify);

const int ledPin = LED_BUILTIN; // pin to use for the LED
const int buttonPin = A0; // set buttonPin

void setup() {
  Serial.begin(115200);
  delay(3000);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT); // use button pin as an input

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName("BUTTON_LED");
  BLE.setLocalName("BUTTON_LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(LEDCharacteristic);
  ledService.addCharacteristic(buttonCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  LEDCharacteristic.writeValue(0);
  buttonCharacteristic.writeValue(1);
  
  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral Started");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // read the current button pin state
      char buttonValue = digitalRead(buttonPin);
    
      // has the value changed since the last read
      bool buttonChanged = (buttonCharacteristic.value() != buttonValue);
    
      if (buttonChanged) {
        // button state changed, update characteristics
        buttonCharacteristic.writeValue(buttonValue);
      }

      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (LEDCharacteristic.written()) {
        if (LEDCharacteristic.value()) {   // any value other than 0
          Serial.println("LED on");
          digitalWrite(ledPin, HIGH);         // will turn the LED on
        } else {                              // a 0 value
          Serial.println(F("LED off"));
          digitalWrite(ledPin, LOW);          // will turn the LED off
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
