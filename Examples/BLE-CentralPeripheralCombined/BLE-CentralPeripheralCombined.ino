/*
  BLE-CentralPeripheralCombined

  This sets up a device to read and listen to a button press 
  and then turns on an LED and plays a noise on a seperate device when the button is pressed
  Communication works both ways so if a button is pressed on one device the other device activates, no matter
  which is central and which is peripheral

  Killer Robots from Outer Space (KROS)
  Peter Schmitt, Rita Tagarao, Logan Setzkorn, Jacob Block 
  2022 November 29
  Based on the ArduinoBLE LED sketch combined with ButtonLED

  This example code is in the public domain.
*/

#include <ArduinoBLE.h> //Bluetooth Low Energy library
#include <VL6180X.h>  // distance sensor library
#include <PCA95x5.h>           // MUX library

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "c87dd0eb-290a-4df5-ba37-ca7b9418b205"
#define BLE_UUID_LED                 "c87dd0eb-290a-4df5-ba37-ca7b9418b206"
#define BLE_UUID_DISTANCE              "c87dd0eb-290a-4df5-ba37-ca7b9418b207"

BLEService ledService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEIntCharacteristic LEDCharacteristic(BLE_UUID_LED, BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEIntCharacteristic distanceCharacteristic(BLE_UUID_DISTANCE, BLERead | BLENotify);

// define which device this is
#define setPeripheral 0   //Can change: 1 is peripheral, 0 is central
char* robotName = "KROS"; //Don't change

// variables for buzz pin, led pin, and distance
#define BUZZ_PIN           2
const int ledPin = LED_BUILTIN;
int oldDistance = 756;

#define SCALING 3

  // objects for the VL6180X TOF distance sensor
  VL6180X sensor;      // distance sensor object
  PCA9535 muxU31;      // MUX object for U31 PCA9535

void setup() {
  Serial.begin(115200);
  delay(3000);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);

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
    ledService.addCharacteristic(distanceCharacteristic);

    // add service
    BLE.addService(ledService);

    // set the initial value for the characeristic:
    LEDCharacteristic.writeValue(0);
    distanceCharacteristic.writeValue(756);
  
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
        // read the current distance
        char distanceValue = sensor.readRangeSingleMillimeters();
    
        // has the value changed since the last read
        bool distanceChanged = (distanceCharacteristic.value() != distanceValue);
    
        if (distanceChanged) {
          // button state changed, update characteristics
          distanceCharacteristic.writeValue(distanceValue);
        }

        // if the remote device wrote to the characteristic,
        // use the value to control the LED:
        if (LEDCharacteristic.written()) {
          if (LEDCharacteristic.value()) {   // any value other than 0
            Serial.println("LED on");
            digitalWrite(ledPin, HIGH);         // will turn the LED on
            tone(BUZZ_PIN, 500);
          } else {                              // a 0 value
            Serial.println(F("LED off"));
            noTone(BUZZ_PIN);
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
  BLECharacteristic distanceCharacteristic = peripheral.characteristic(BLE_UUID_DISTANCE);

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

  if (!distanceCharacteristic) {
    Serial.println("Peripheral does not have distance characteristic!");
    peripheral.disconnect();
    return;
  } else if (!distanceCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable distance characteristic!");
    peripheral.disconnect();
    return;
  } else if (!distanceCharacteristic.canSubscribe()) {
    Serial.println("Peripheral does not allow distance subscriptions (notify)");
  } else if(!distanceCharacteristic.subscribe()) {
    Serial.println("Subscription failed!");
  } else {
    Serial.println("Connected to distance characteristic");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // read the distance
    int distanceState = sensor.readRangeSingleMillimeters();

    if (oldDistance != distanceState) {
      // distance changed
      oldDistance = distanceState;

      if (!distanceState) {
        Serial.println("Distance changed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeValue((byte)0x01);
      } else {
        Serial.println("Distance changed");

        // button is released, write 0x00 to turn the LED off
        ledCharacteristic.writeValue((byte)0x00);
      }
    }
        if(distanceCharacteristic && distanceCharacteristic.canRead() && distanceCharacteristic.valueUpdated()) {
      int peripheralDistance;
      distanceCharacteristic.readValue(&peripheralDistance, sizeof(int));
      if(!peripheralDistance) {
        digitalWrite(ledPin, HIGH);
        tone(BUZZ_PIN, 1000);
      } else {
        digitalWrite(ledPin, LOW);
        noTone(BUZZ_PIN);
      }
      Serial.print("peripheralDistance = ");
      Serial.println(peripheralDistance);
    }

  }

  Serial.println("Peripheral disconnected");
}
