/*
 * Filename:
 * Purpose:
 * 
 * Contributors:
 * Date:
 */

//Libraries
#include <ArduinoBLE.h>


//UUIDs
#define BLE_UUID_PERIPHERAL          "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_LED                 "19B10001-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_BUTTON              "19B10012-E8F2-537E-4F6C-D104768A1214"

//Objects
BLEService ledService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy LED Service
  // Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
  BLEByteCharacteristic LEDCharacteristic(BLE_UUID_LED, BLERead | BLEWrite);
  // create button characteristic and allow remote device to get notifications
  BLEByteCharacteristic buttonCharacteristic(BLE_UUID_BUTTON, BLERead | BLENotify);

//Pins
const int ledPin = LED_BUILTIN; // pin to use for the LED
const int buttonPin = A0; // set buttonPin

//Variables
int mode = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  //Bluetooth Stuff
    pinMode(ledPin, OUTPUT);    // set LED pin to output mode
    pinMode(buttonPin, INPUT);  // use button pin as an input
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

  //Main sequence stuff


}

void loop() {
  //Declare variables

  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  //Switch between different modes
  switch(mode) {
    case 1:   //Standby
      //scan and detect
      break;
    case 2:   //Chase or Attack
      //PID
      //Alert function
      break;
    case 3:   //Only Alert
      //Alert only, arrived at obstacle or cannot reach
      break;
    case 4:   //Withdraw
      //After 5s or so, withdraw from target
      break;
    default:  //Idle (all operations suspended)
      //All operations off and safe
      //Add message to tft saying idle
    break;
  }
}
