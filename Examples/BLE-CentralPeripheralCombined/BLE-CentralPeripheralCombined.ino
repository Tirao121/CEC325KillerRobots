/*
  BLE-CentralPeripheralCombined

  This sets up a device to read and listen to a button press 
  and then turns on an LED and plays a noise on a seperate device when the button is pressed

  Killer Robots from Outer Space (KROS)
  Peter Schmitt, Rita Tagarao, Logan Setzkorn, Jacob Block 
  2022 November 29
  Based on the ArduinoBLE LED sketch combined with ButtonLED

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "c87dd0eb-290a-4df5-ba37-ca7b9418b205"
#define BLE_UUID_LED                 "c87dd0eb-290a-4df5-ba37-ca7b9418b206"
#define BLE_UUID_BUTTON              "c87dd0eb-290a-4df5-ba37-ca7b9418b207"

BLEService ledService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic LEDCharacteristic(BLE_UUID_LED, BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEByteCharacteristic buttonCharacteristic(BLE_UUID_BUTTON, BLERead | BLENotify);

// define which device this is
#define setPeripheral 0   //Can change: 1 is peripheral, 0 is central
char* robotName = "KROS"; //Don't change

// variables for button
#define RIGHT_BUTTON_PIN   A0
#define BUZZ_PIN           2
const int buttonPin = RIGHT_BUTTON_PIN;
const int ledPin = LED_BUILTIN;
int oldButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(3000);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT); // use button pin as an input
  pinMode(BUZZ_PIN, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  else {
    Serial.println("Bluetooth® Low Energy Central - LED control");
  }

  if (setPeripheral == 1) {
   // set advertised local name and service UUID:
    BLE.setDeviceName(robotName);
    BLE.setLocalName(robotName);
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
  else {
    // start scanning for peripherals
    Serial.println("Connecting to Peripheral");
    BLE.scanForUuid(BLE_UUID_PERIPHERAL);
    Serial.println("Connected to Peripheral");
  }
}

void loop() {
  if (setPeripheral ==1) {
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
            tone(BUZZ_PIN, 500, 100);
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
  else {
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

      if (peripheral.localName() != robotName) {
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
    Serial.println(peripheral);
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
