/*
 * Filename: KillerRobotsMain
 * Purpose: Sentinal Robots designed to detect and 'attack' intruders
 * 
 * Contributors:
 * Peter Schmitt
 * Rita Tagarao
 * Logan Setzkorn
 * Jacob Block 
 * Date:
 */

//Libraries
#include <SPI.h>
#include "WiFiNINA.h"  // for A4-A7
#include <ArduinoBLE.h> //Bluetooth Low Energy library
#include <VL6180X.h>  // distance sensor library
#include <PeakDetection.h>
#include <Adafruit_NeoPixel.h> // NeoPixel library
#include <PCA95x5.h>           // MUX library
#include <Servo.h>             // Servo
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789 display

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------
#define BLE_UUID_PERIPHERAL          "c87dd0eb-290a-4df5-ba37-ca7b9418b205"
#define BLE_UUID_PMODE              "c87dd0eb-290a-4df5-ba37-ca7b9418b206"
#define BLE_UUID_CMODE              "c87dd0eb-290a-4df5-ba37-ca7b9418b207"

BLEService modeService(BLE_UUID_PERIPHERAL); // Bluetooth® Low Energy Mode Service

// create mode characteristic and allow remote device to get notifications
BLEByteCharacteristic pModeCharacteristic(BLE_UUID_PMODE, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic cModeCharacteristic(BLE_UUID_CMODE, BLERead | BLEWrite | BLENotify);

//Pins & Defines
#define RIGHT_BUTTON_PIN   A0
#define BUZZ_PIN           2
#define SERVO_PIN          9
#define NEO_PIN           17  // WARNING! THIS IS GPIO NOT D NUMBER for NeoPixels
#define NEO_COUNT         18  // number of NeoPixels
#define NEO_MAG           100 // NeoPixel default magnitude
#define AIN1              0  // Motor driver A1
#define AIN2              1  // Motor driver A2
#define BIN1              8  // Motor driver B2
#define BIN2              A2  // Motor driver B1
#define TFT_CS            4  // display chip select
#define TFT_RST           A3 // display reset
#define TFT_DC            3  // display D/C
const int ledPin = LED_BUILTIN;

#define SCALING 3

//Objects
  // Declare our NeoPixel strip object:
  Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
  // objects for the VL6180X TOF distance sensor
  VL6180X sensor;      // distance sensor object
  PCA9535 muxU31;      // MUX object for U31 PCA9535
Servo myservo;
  // Display
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  GFXcanvas16 canvas(240,135);
PeakDetection peakDetection; // create PeakDetection object

//Variables
#define setPeripheral 0   //Can change: 1 is peripheral, 0 is central
char* robotName;
byte pMode = 1;
byte cMode = 1;
int mode = 1;
double proxThreashold = 100;
  //PID Constants
  float KP = 7;  // proportional control gain
  float KI = .001; // integral gain
  float KD = 10;    // derivative gain
int proximity = 0;
int motorspeed = 50;
double angle = 90;       //swivel angle when change detected - default 90
double data = 0.0;
int alertCounter = 0;
unsigned long lastTime = millis();
unsigned long curTime;
int turnNeeded = 1;
float proxThreshold = 400.0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
Serial.println("Starting...");

  //Distance Sensor setup
  Wire.begin();
  muxU31.attach(Wire, 0x20);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0x1CFF);  // 1 is input, see schematic to get upper and lower bytes
  muxU31.write(PCA95x5::Port::P09, PCA95x5::Level::H);  // enable VL6180 distance sensor
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(1000);

  //NeoPixel setup
  strip.begin();
  strip.clear();  // Set all pixel values to zero
  strip.show();   // Write values to the pixels

  pinMode(BUZZ_PIN, OUTPUT);    //Buzzer setup
  
  //Servo Setup
    //Distance sensor swivel
    myservo.attach(SERVO_PIN);
    myservo.write(90);
    //Wheel servos
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

  //Display setup (Must be after NeoPixels)
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, LOW); // disable TFT in case it was left running
  tft.init(135, 240);           // Init ST7789 240x135
  Serial.println("Display initialized");
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);

  //Peak detection setup
  peakDetection.begin(75, 4, 0.025); // sets the lag, threshold and influence
  peakDetection.setEpsilon(0.02);

  //Bluetooth setup
    if (setPeripheral == 1) {
    robotName = "KROSP"; //Don't change
  }
  else {
    robotName = "KROSC"; //Don't change
  }

  // begin Bluetooth initialization
  if (!BLE.begin()) {
    //Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  else {
    //Serial.println("Bluetooth® Low Energy Central - Mode control");
  }

  //if peripheral device
  if (setPeripheral == 1) {
   // set advertised local name and service UUID:
    BLE.setDeviceName(robotName);
    BLE.setLocalName(robotName);
    BLE.setAdvertisedService(modeService);

    // add the characteristic to the service
    modeService.addCharacteristic(pModeCharacteristic);
    modeService.addCharacteristic(cModeCharacteristic);

    // add service
    BLE.addService(modeService);

    // set the initial value for the characeristic:
    pModeCharacteristic.writeValue(pMode);
    cModeCharacteristic.writeValue(cMode);
  
    // start advertising
    BLE.advertise();

    //Serial.println("BLE Mode Peripheral Started");
  }
  //if central device
  else {
    // start scanning for peripherals
    //Serial.println("Connecting to Peripheral");
    BLE.scanForUuid(BLE_UUID_PERIPHERAL);
    //Serial.println("Connected to Peripheral");
  }
}

void loop() {
  //Declare variableS
  if (setPeripheral == 1) {
    pMode = (byte)mode;
  }
  else {
    cMode = (byte)mode;
  }
  //Switch between different modes
  switch(mode) {
    case 1:   //Standby
     // Serial.println("I'm in mode 1");
      angle = standby();
      lastTime = millis();
      turnNeeded = 1;
      break;
    case 2: 
      //Serial.println("I'm in mode 2");
      //To prevent turning forever loop
      if (turnNeeded == 1) {
        turn(angle);
      }
      attack();
      Alert();

      curTime = millis();
      if (curTime - lastTime >= 8000) {
        mode = 3;   //after 5 seconds, implement withdraw function
        lastTime = millis();
      }
      break;
    case 3:   //Withdraw
      //Serial.println("I'm in mode 3");
      //After 5s or so, withdraw from target
      withdraw();
      curTime = millis();
      if (curTime - lastTime >= 10000) {
        mode = 1;   //after 10 seconds, return to standby
      }
      break;
    default:  //Idle (all operations suspended)
      //All operations off and safe
        idle();
      //Add message to tft saying idle?
    break;
  }
 // Serial.println(mode);
 // Serial.println("\t");
  //Serial.println(angle);

  //Wait until both devices in the same mode
  if (setPeripheral == 1) {
    loopPeripheral();
  }
  else {
    loopCentral();
  }
}

/* Detects change in radius, changes global var "mode" to alert if something detected
  and returns current angle swivel is in. */
double standby() {
  //Detect change in proximity
    //Use peak detect? Problem with swivel?
    //detects wheels at 155 and 1 (margin of 1 degree)
    //delay needed?
    //have detect peak for multiple values first?
  //Freeze wheels
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
  double pos = 0.0;
  int distance = 0;
  for (pos = 45; pos <= 155; pos += .5) {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      data = (float)distance/765-1;  // 16-bit int -> +/- 1.0 range
      float stdpt = peakDetection.add(data); // adds a new data point
      int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    //Only negative peak - enters field, not move away
    if(peak == -1) {
      //Serial.println(pos);
      delay(1000);
      mode = 2;
      return pos;
    } else {
      mode = 1;
    }
  }
  for (pos = 155; pos >= 15; pos -= .5) {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      data = (float)distance/765-1;  // 16-bit int -> +/- 1.0 range
    float stdpt = peakDetection.add(data); // adds a new data point
    int peak = peakDetection.getPeak();//*5+75; // returns 0, 1 or -1
    //Only negative peak - enters field, not move away
    if(peak == -1) {
      mode = 2;
      return pos;
      //Serial.println(pos);
    } else {
      mode = 1;
    }
  }  
  return pos;
}

// PID control
float pidControl(float error) {
  static float cumError = 0.0;
  static int lastError = 0;
  float d = KD*(error - lastError);
  cumError += KI*(float)error;
  //Serial.print(cumError);
  //Serial.print(",");
  //Serial.print(d);
  //Serial.print(",");
  lastError = error; 
  //Serial.println(error);
  return(KP*error + (float)cumError + d);
}


void attack() {
  proximity = sensor.readRangeSingleMillimeters();
  float distanceError = proximity-proxThreshold;
  float  motorSpeed = pidControl(distanceError);
  //Serial.println(distanceError);
  //Serial.println(proximity);
  //Serial.println(proxThreshold);  

    int absSpeed = abs(motorSpeed);
  if(motorSpeed > 0) {
    analogWrite(AIN1, absSpeed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, absSpeed);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, absSpeed);
    analogWrite(BIN1, absSpeed);
    analogWrite(BIN2, 0);
  }
}


//alert function - must be called continuously in order to work
void Alert(){
  //Buzzer 
  if(alertCounter == 0){
    tone(BUZZ_PIN, 466.16, 500);
  }
   tone(BUZZ_PIN, 200, 500);
  if(alertCounter == 5){
  }

  //Neopixels
  for(int ii = 0; ii < NEO_COUNT; ii++) {
    if(alertCounter == 0){
      strip.setPixelColor(ii, strip.Color(100,0,0));
      }
      else if(alertCounter == 5){
        strip.setPixelColor(ii, strip.Color(0,0,100));
    }
  }
  strip.show();

  //display
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.print(" _                   _");
  tft.println(" _( )                 ( )_");
  tft.println("(_, |      __ __      | ,_)");
  tft.println("   \'\    /  ^  \    /'/");
  tft.println("    '\'\,/\      \,/'/'");
  tft.println("      '\| []   [] |/'");
  tft.println("        (_  /^\  _)");
  tft.println("          \  ~  /");
  tft.println("          /HHHHH\\");
  tft.println("        /'/{^^^}\'\ ");
  tft.println("    _,/'/'  ^^^  '\'\,_");
  tft.println("   (_, |           | ,_)");
  tft.println("     (_)           (_)");

  alertCounter++;
  if(alertCounter >= 10){
    alertCounter = 0;
  }
}

void withdraw() { 
  //waits for 5 seconds
  //slowly backs away while silencing buzzers, blink lights?
  //ends with turning off lights and returning to standby mode

  //buzzer off at 6s
  if (curTime - lastTime >= 3000) {
    noTone(BUZZ_PIN);
  }

  //servos after 4s
  if (curTime - lastTime <= 4000) {
       //Backs away 
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 100);
    analogWrite(BIN1, 100);
    analogWrite(BIN2, 0);
    analogWrite(AIN1, 100);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 100);
  } else {
      //breaks
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }

  //display blank at 8s
  if (curTime - lastTime >= 8000) {
    tft.fillScreen(ST77XX_BLACK);
  }

  //Neopixels off at 9s
  if (curTime - lastTime >= 9000) {
    strip.clear();
    strip.show();
  } else {
    for(int ii = 0; ii < NEO_COUNT; ii++) {
      strip.setPixelColor(ii, strip.Color(100,0,0));
    }
  }
}

void idle(){
  //lights
  strip.clear();
  strip.show();
  //buzzer
  noTone(BUZZ_PIN);
  //tft
  digitalWrite(TFT_CS, LOW);
  //servos
    myservo.write(90);
    //Wheel servos
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
    pinMode(BIN1, INPUT);
    pinMode(BIN2, INPUT);
  //flashing led
  int i = 0;
  while(true){
    if(i % 2 == 0){
      digitalWrite(ledPin, HIGH);
     }else {
      digitalWrite(ledPin, LOW);    
     }
    i++;
  }
       
}

void turn(double victimAngle) {
  double centerD = 104.0; //depends on board. Must be calibrated
  int centerI = 104; //depends on board. Must be calibrated
  double multiplier = 8.0;
  //Serial.println("Im Turning");
  // Serial.println(victimAngle);
  if (victimAngle < centerD) {
  analogWrite(BIN2, 0); //R forward
  analogWrite(AIN1, 255/2); //L forward
  analogWrite(AIN2, 0); //L back
  analogWrite(BIN1, 0); //R back
 // Serial.println((centerD - victimAngle));
  delay((centerD - victimAngle) * (multiplier)); //done through interation. Could be diff for diff board
  }
  if (victimAngle > centerD) {
  analogWrite(BIN2, 0); //R forward
  analogWrite(AIN1, 0); //L forward
  analogWrite(AIN2, 255/2); //L back
  analogWrite(BIN1, 0); //R back
  //delay(2000);
  //Serial.println((victimAngle - centerD));
  delay((victimAngle - centerD) * (multiplier)); //done through interation. Could be diff for diff board
  }
  analogWrite(BIN2, 0); //R forward
  analogWrite(AIN1, 0); //L forward
  analogWrite(AIN2, 0); //L back
  analogWrite(BIN1, 0); //R backf
  myservo.write(centerI); //May need to be calibrated. It should be in the center
 

 // if turn is complete by the end of this function
  if (sensor.readRangeSingleMillimeters() < 765) {
  turnNeeded = 0;
  } else {
  mode = 1;
  turnNeeded = 1; 
  }
 delay(1000);

}

//The loop function for the peripheral device. It will connect to central and wait until both central and peripheral are in the same mode
void loopPeripheral() {
  // listen for Bluetooth® Low Energy peripherals to connect:
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central) {
      //Serial.print("Connected to central: ");
      // print the central's MAC address:
      //Serial.println(central.address());

      // while the central is still connected to peripheral:
      while (central.connected()) {
        // initialize the current mode
        pModeCharacteristic.writeValue(pMode);
        cModeCharacteristic.writeValue(cMode);
        byte peripheralModeValue = (byte)pModeCharacteristic.value(); 
        byte centralModeValue = (byte)cModeCharacteristic.value();
        
        //Check if both modes match and delay the peripheral device if needed
        if (peripheralModeValue != centralModeValue) {
          // Modes don't match, check which one needs to wait
          if(peripheralModeValue == 1) {
            if (centralModeValue == 3) {
              //If central is 3 and peripheral is 1, then peripheral needs to wait for central to catch up
              while(!cModeCharacteristic.written()) {
                //Wait until central characteristic is updated
              }
              centralModeValue = cModeCharacteristic.value();
            }
            if (centralModeValue == 2) {
              //If central is 1 and peripheral is 3, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else if (centralModeValue != 2 || centralModeValue != 3) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
          else if(peripheralModeValue == 2) {
            if(centralModeValue == 1) {
              //If central is 1 and peripheral is 2, then peripheral needs to wait for central to catch up
              while (!cModeCharacteristic.written()) {
                //Wait until central characteristic is updated
              }
              centralModeValue = cModeCharacteristic.value();
            }
            if (centralModeValue == 3) {
              //If central is 3 and peripheral is 2, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else if (centralModeValue != 1 || centralModeValue != 3) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
          else if (peripheralModeValue == 3) {
            if (centralModeValue == 2) {
              //If central is 2 and peripheral is 3, then peripheral needs to wait for central to catch up
              while(!cModeCharacteristic.written()) {
                //Wait until central characteristic is updated
              }
              centralModeValue = cModeCharacteristic.value();
            }
            if (centralModeValue == 1) {
              //If central is 2 and peripheral is 3, then central needs to wait for peripheral to catch up, peripheral needs to keep running
            }
            else if (centralModeValue != 2 || centralModeValue != 1) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
        }

        // Modes should now match, can continue out of bluetooth and back to normal switch
        //Serial.println("Modes Match");
        //Serial.print("Central Mode: ");
        //Serial.println(centralModeValue);
        //Serial.print("Peripheral Mode: ");
        //Serial.println(peripheralModeValue);
      }

      // when the central disconnects, print it out:
      //Serial.print(F("Disconnected from central: "));
      //Serial.println(central.address());
    }
}

//The loop function for the central device. It will connect to peripheral and wait until both central and peripheral are in the same mode
void loopCentral () {
  // check if a peripheral has been discovered
    BLEDevice peripheral = BLE.available();

    if (peripheral) {
      // discovered a peripheral, print out address, local name, and advertised service
      //Serial.print("Found ");
      //Serial.print(peripheral.address());
      //Serial.print(" '");
      //Serial.print(peripheral.localName());
      //Serial.print("' ");
      //Serial.print(peripheral.advertisedServiceUuid());
      //Serial.println();

      if (peripheral.localName() != "KROSP") {
        //Serial.print("Wrong local name: ");
        //Serial.println(peripheral.localName());
        return;
      }

      // stop scanning
      BLE.stopScan();

      // connect to the peripheral
  //Serial.println("Connecting ...");

  if (peripheral.connect()) {
    //Serial.println("Connected");
  } else {
    //Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  //Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    //Serial.println("Attributes discovered");
  } else {
    //Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the Mode characteristic
  BLECharacteristic pModeCharacteristic = peripheral.characteristic(BLE_UUID_PMODE);
  BLECharacteristic cModeCharacteristic = peripheral.characteristic(BLE_UUID_CMODE);

  if (!pModeCharacteristic) {
    //Serial.println("Peripheral does not have pMode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!pModeCharacteristic.canRead()) {
    //Serial.println("Peripheral does not have a readable pMode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!pModeCharacteristic.canSubscribe()) {
    //Serial.println("Peripheral does not allow pMode subscriptions (notify)");
  } else if(!pModeCharacteristic.subscribe()) {
    //Serial.println("Subscription failed!");
  } else {
    //Serial.println("Connected to peripheral mode characteristic");
  }

  if (!cModeCharacteristic) {
    //Serial.println("Peripheral does not have cMode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!cModeCharacteristic.canRead()) {
    //Serial.println("Peripheral does not have a readable cMode characteristic!");
    peripheral.disconnect();
    return;
  } else if (!cModeCharacteristic.canSubscribe()) {
    //Serial.println("Peripheral does not allow cMode subscriptions (notify)");
  } else if(!cModeCharacteristic.subscribe()) {
    //Serial.println("Subscription failed!");
  } else {
    //Serial.println("Connected to central mode characteristic");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

      // initialize the current mode
      byte centralModeValue;
      cModeCharacteristic.readValue(centralModeValue);
      byte peripheralModeValue;
      pModeCharacteristic.readValue(peripheralModeValue);

     // Serial.println("Initialized ModeValues");
      //Serial.print("Central: ");
      //Serial.println(centralModeValue);
      //Serial.print("Peripheral: ");
      //Serial.println(peripheralModeValue);
        
      //Check if both modes match and delay the central device if needed
      if (centralModeValue != peripheralModeValue) {
          // while modes do not match, check which one needs to wait
          if(centralModeValue == 1) {
            if(peripheralModeValue == 3) {
              //If peripheral is 3 and central is 1, then peripheral needs to wait for center to catch up, center needs to keep running
              while(!pModeCharacteristic.written()) {
                //Wait until peripheral characteristic is updated
              }
              pModeCharacteristic.readValue(peripheralModeValue);
            }
            if (peripheralModeValue == 2) {
              //If peripheral is 2 and central is 1, then central needs to wait for peripheral to catch up
            }
            else if (peripheralModeValue != 2 || peripheralModeValue != 3) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
          else if(centralModeValue == 2) {
            if (peripheralModeValue == 1) {
              //If peripheral is 1 and central is 2, then central needs to wait for peripheral to catch up
              while(!pModeCharacteristic.written()) {
                //Wait until peripheral characteristic is updated
              }
              pModeCharacteristic.readValue(peripheralModeValue);
            }
            if (peripheralModeValue == 3) {
              //If peripheral is 3 and central is 2, then peripheral needs to wait for central to catch up, central needs to keep running
            }
            else if (peripheralModeValue != 2 || peripheralModeValue != 3) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
          else if (centralModeValue == 3) {
            if (peripheralModeValue == 2) {
              //If peripheral is 2 and central is 3, then central needs to wait for peripheral to catch up
              while(!pModeCharacteristic.written()) {
                //Wait until peripheral characteristic is updated
              }
              pModeCharacteristic.readValue(peripheralModeValue);
            }
            if (peripheralModeValue == 1) {
              //If peripheral is 1 and central is 3, then peripheral needs to wait for central to catch up, central needs to keep running
            }
            else if (peripheralModeValue != 2 || peripheralModeValue != 1) {
              //Serial.println("These modes don't make sense");
              //Serial.print("Central Mode: ");
              //Serial.println(centralModeValue);
              //Serial.print("Peripheral Mode: ");
              //Serial.println(peripheralModeValue);
            }
          }
        }

        // Modes should now match, can continue out of bluetooth and back to normal switch
        //Serial.println("Modes Match");
        //Serial.print("Central Mode: ");
        //Serial.println(centralModeValue);
        //Serial.print("Peripheral Mode: ");
        //Serial.println(peripheralModeValue);

      // peripheral disconnected, start scanning again
      BLE.scanForUuid(BLE_UUID_PERIPHERAL);
    }
    //Serial.println(peripheral);
  }
}
