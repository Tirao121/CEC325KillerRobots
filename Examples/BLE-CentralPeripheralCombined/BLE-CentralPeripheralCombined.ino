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

//Libraries

#include <ArduinoBLE.h> //Bluetooth Low Energy library

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "c87dd0eb-290a-4df5-ba37-ca7b9418b205"
#define BLE_UUID_MODE              "c87dd0eb-290a-4df5-ba37-ca7b9418b207"

BLEService modeService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy Mode Service

// create mode characteristic and allow remote device to get notifications
BLEByteCharacteristic modeCharacteristic(BLE_UUID_MODE, BLERead | BLEWrite | BLENotify);

// define which device this is
#define setPeripheral 0   //Can change: 1 is peripheral, 0 is central

// variables for bluetooth
byte oldMode = 1;

//Variables
byte mode = 3;

void setup() {
  Serial.begin(115200);
  delay(3000);

  while (!Serial) {
    
  }

  if (setPeripheral == 0) {
  static char* robotName = "KROSP"; //Don't change
}
else {
  static char* robotName = "KROSC"; //Don't change
}

  // begin Bluetooth initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  else {
    Serial.println("Bluetooth® Low Energy Central - Mode control");
  }

  //if peripheral device
  if (setPeripheral == 1) {
   // set advertised local name and service UUID:
    BLE.setDeviceName(robotName);
    BLE.setLocalName(robotName);
    BLE.setAdvertisedService(modeService);

    // add the characteristic to the service
    modeService.addCharacteristic(modeCharacteristic);

    // add service
    BLE.addService(modeService);

    // set the initial value for the characeristic:
    modeCharacteristic.writeValue(mode);
  
    // start advertising
    BLE.advertise();

    Serial.println("BLE Mode Peripheral Started");
  }
  //if central device
  else {
    // start scanning for peripherals
    Serial.println("Connecting to Peripheral");
    BLE.scanForUuid(BLE_UUID_PERIPHERAL);
    Serial.println("Connected to Peripheral");
  }
}

void loop() {
  if (setPeripheral == 0) {
    static char* robotName = "KROSP"; //Don't change
  }
  else {
    static char* robotName = "KROSC"; //Don't change
  }
  
  //if peripheral device
  if (setPeripheral == 1) {
    loopPeripheral();
    delay(5000);
    mode = 2;
  }
  //if central device
  else {
    loopCentral();
  }
}


//The loop function for the peripheral device. It will connect to central and wait until both central and peripheral are in the same mode
void loopPeripheral() {
  // listen for Bluetooth® Low Energy peripherals to connect:
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central) {
      Serial.print("Connected to central: ");
      // print the central's MAC address:
      Serial.println(central.address());

      // while the central is still connected to peripheral:
      while (central.connected()) {
        // initialize the current mode
        byte peripheralModeValue = modeCharacteristic.value(); //some error here
        byte centralModeValue = 3;
        Serial.println("Set mode values");
        
        //Check if both modes match and delay the peripheral device if needed
        while (oldMode != modeCharacteristic.value()) {
          Serial.println("Made it to oldMode while loop");
          // while peripheral mode changed, check which one needs to wait
          if(centralModeValue == 1) {
            if(peripheralModeValue == 2) {
              //If central is 1 and peripheral is 2, then peripheral needs to wait for central to catch up
              Serial.println("Waiting for central to catch up");
              delay(100);
            }
            else if (peripheralModeValue == 3) {
              //If central is 1 and peripheral is 3, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
          else if(centralModeValue == 2) {
            if(peripheralModeValue == 1) {
              //If central is 2 and peripheral is 1, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else if (peripheralModeValue == 3) {
              //If central is 2 and peripheral is 3, then peripheral needs to wait for central to catch up
              Serial.println("Waiting for central to catch up");
              delay(100);
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
          else if (centralModeValue == 3) {
            if (peripheralModeValue == 2) {
              //If central is 3 and peripheral is 2, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else if (peripheralModeValue == 1) {
              //If central is 3 and peripheral is 1, then peripheral needs to wait for central to catch up
              Serial.println("Waiting for central to catch up");
              delay(100);
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
        }

        // Modes should now match, can continue out of bluetooth and back to normal switch
        Serial.println("Modes Match");
      }

      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
    }
}

//The loop function for the central device. It will connect to peripheral and wait until both central and peripheral are in the same mode
void loopCentral () {
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

      if (peripheral.localName() != "KROSP") {
        Serial.print("Wrong local name: ");
        Serial.println(peripheral.localName());
        return;
      }

      // stop scanning
      BLE.stopScan();

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

  // retrieve the Mode characteristic
  BLECharacteristic modeCharacteristic = peripheral.characteristic(BLE_UUID_MODE);

  if (!modeCharacteristic) {
    Serial.println("Peripheral does not have mode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!modeCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable mode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!modeCharacteristic.canSubscribe()) {
    Serial.println("Peripheral does not allow mode subscriptions (notify)");
  } else if(!modeCharacteristic.subscribe()) {
    Serial.println("Subscription failed!");
  } else {
    Serial.println("Connected to distance characteristic");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

      // initialize the current mode
      byte centralModeValue = mode;
      byte peripheralModeValue = (char)*modeCharacteristic.value();
        
      //Check if both modes match and delay the central device if needed
      while (centralModeValue != peripheralModeValue) {
          // while modes do not match, check which one needs to wait
          if(peripheralModeValue == 1) {
            if(centralModeValue == 2) {
              //If peripheral is 1 and central is 2, then central needs to wait for peripheral to catch up
              Serial.println("Waiting for peripheral to catch up");
              delay(100);
            }
            else if (centralModeValue == 3) {
              //If peripheral is 1 and central is 3, then peripheral needs to wait for center to catch up, center needs to keep running
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
          else if(peripheralModeValue == 2) {
            if(centralModeValue == 1) {
              //If peripheral is 2 and central is 1, then peripheral needs to wait for central to catch up, central needs to keep running
            }
            else if (centralModeValue == 3) {
              //If peripheral is 2 and central is 3, then central needs to wait for peripheral to catch up
              Serial.println("Waiting for peripheral to catch up");
              delay(100);
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
          else if (peripheralModeValue == 3) {
            if (centralModeValue == 2) {
              //If peripheral is 3 and central is 2, then peripheral needs to wait for central to catch up, central needs to keep running
            }
            else if (centralModeValue == 1) {
              //If peripheral is 3 and central is 1, then central needs to wait for peripheral to catch up
              Serial.println("Waiting for peripheral to catch up");
              delay(100);
            }
            else {
              Serial.println("These modes don't make sense");
              Serial.print("Central Mode: ");
              Serial.println(centralModeValue);
              Serial.print("Peripheral Mode: ");
              Serial.println(peripheralModeValue);
            }
          }
        }

        // Modes should now match, can continue out of bluetooth and back to normal switch
        Serial.println("Modes Match");

      // peripheral disconnected, start scanning again
      BLE.scanForUuid(BLE_UUID_PERIPHERAL);
    }
    Serial.println(peripheral);
  }
}
