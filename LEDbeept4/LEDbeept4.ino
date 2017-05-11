/*
  This is the code for the Beepbeep
  the Beepbeep is Valery's robot
  :info for the sd card reader:
   MOSI - pin 11
   MISO - pin 12
   CLK - pin 13
   CS  - pin 10
*/

//look it's the includes
#include <CurieBLE.h>
#include "CurieIMU.h"
#include <SPI.h>
#include <SD.h>
#include <ASCII.h>
#include <CurieIMU.h>
#include <stdio.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <MadgwickAHRS.h>
//data log stuff
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
rgb_lcd lcd;
//set up the bluetooth object and set the services being used
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service, this service is used for sending data from phone to arduino

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


//below are the constants llike the pins
//const int ledPin = 13; // pin to use for the LED
//const int motorR = 9;
//const int motorL = 3;
//motor L pins
const int enL = 5; //PWM enA 
const int in1L = A1;
const int in2L = A5;
//I need the beep beep to see which pins are actually availible.
//fix the pin numbers later should still compile
//motor R pins
const int enR = 6; //PWM enB
const int in3R = A3;
const int in4R = A4;

const int beepPin = 8;
const int frsensor = 2;
const int flsensor = 7;
const int freq = 2000; //the base frrequency for the beeps
const int chipSelect = 10;
const int blockFreq = 4200; //frequncy that plays when there is something blocking the beep beep
const int speedmothafucker = 100;//speed from 0-127 at which the motors will go standardly
//below are variables,, they change
int numLines = 0;
String stepString = "" ;
String dataString = "";
String dataStringP = "";
String dataStringR = "";
String dataStringH = "";
int myData;
bool task;
bool activeTask;
int ambientIR;                // variable to store the IR coming from the ambient
int obstacleIR;               // variable to store the IR coming from the object
int value[10];                // variable to store the IR values
int distance;                 // variable that will tell if there is an obstacle or not
bool proxr;  //variable goes low if something is infront of beepBeep
bool proxl; //variable goes low if something is infront of beepBeep
int axRaw, ayRaw, azRaw;         // raw accelerometer values
int gxRaw, gyRaw, gzRaw;         // raw gyro values





int r = random(0, 245);
  int g = random(0, 245);
  int b = random(0, 245);
void setup() {
  //lcd screen stuff

  lcd.begin(16, 2);
randomSeed(analogRead(0));
  lcd.setRGB(r, g, b);
  lcd.print("The beepBeep is");
  lcd.setCursor(0, 1);
  lcd.print("prepping 4 journey");
  lcd.setCursor(0, 0);

  Serial.begin(9600);
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();


  Serial.println("Arduino101/IntelCurie/Accelerometer/Evothings Example Started");
  Serial.println("Serial rate set to 9600");
  Serial.println("Initializing IMU device...");
  // Set the accelerometer range to 2G
  //set the pins
  pinMode(beepPin, OUTPUT);
  //pinMode(motorR, OUTPUT);
  //pinMode(motorL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);
  pinMode(in3R, OUTPUT);
  pinMode(in4R, OUTPUT);

  //the two below are for the Ir sensors
  pinMode(frsensor, INPUT_PULLUP);
  pinMode(flsensor, INPUT_PULLUP);
  // and the sound sensor should go in here.

  // set advertised local name and service UUID:
  blePeripheral.setLocalName("LED");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);

  // set the initial value for the characeristic:
  switchCharacteristic.setValue(0);
  // All characteristics should be initialized to a starting value prior
  // using them.
  // begin advertising BLE service:
  blePeripheral.begin();
  Serial.println("BLE LED Peripheral");
  Serial.print("Initializing SD card...");

  // done with bluetooth onto SD card
  //first see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
    //actually beep like a motherfucker to indacate this
    tone(beepPin, 4 * freq);
    delay(250);
    noTone(beepPin);
    tone(beepPin, 1.5 * freq);
    delay(200);
    noTone(beepPin);
  }
  lcd.clear();
}
//this is for keeping track of number of steps saved in a task
int steps = 0;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
unsigned long microsNow;
void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {

      //first check to see if beepBeep is too close to an object
      proxr = digitalRead(frsensor);
      proxl = digitalRead(flsensor);
      if (proxr == LOW || proxl == LOW) {
        //if it is beep instead of moving
        //digitalWrite(13, HIGH);
        Serial.println("it did the thing");
        tone(beepPin, 4200);
        delay(256);
        noTone(beepPin);
        //digitalWrite(13, LOW);
      }
      r = random(0, 245);
      g = random(0, 245);
      b = random(0, 245);
      lcd.setRGB(r, g, b);
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        //display the value recived via bluetooth
        Serial.print(switchCharacteristic.value());
        //write the command to the datafile so we know what's going on
        File dataFile = SD.open("dataa.txt", FILE_WRITE);
        dataString = String(switchCharacteristic.value());
        dataFile.print(dataString);
        dataFile.close();
        //if a task is being recorded the write the comand to the SD card
        if (task == HIGH) {
          File taskFile = SD.open("taska.txt", FILE_WRITE);
          stepString = String(switchCharacteristic.value());
          Serial.println(stepString);
          taskFile.println(stepString);
          taskFile.close();
          steps++;
        }
        //check again to see if too close to an object because yolo

        proxr = digitalRead(frsensor);
        proxl = digitalRead(flsensor);
        if (proxr == LOW || proxl == LOW) {
          digitalWrite(13, HIGH);
          Serial.println("it did the thing");
          tone(beepPin, 4200);
          delay(256);
          noTone(beepPin);
          digitalWrite(13, LOW);

        }
        else {
          //depending on the value that was sent do the required thing
          switch (switchCharacteristic.value()) {
            case 0:
              //stop everything and go backwards
              //analogWrite(motorR, 0);
              //analogWrite(motorL, 0);
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, HIGH);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, HIGH);
              analogWrite(enR, speedmothafucker);
              analogWrite(enL, speedmothafucker);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              delay(500);
              noTone(beepPin);
              // now turn off motors
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, LOW);
              break;
            case 1:
              //go go beepBeep go!
              //analogWrite(motorR, speedmothafucker);
              //analogWrite(motorL, speedmothafucker);
              digitalWrite(in1L, HIGH);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, HIGH);
              digitalWrite(in4R, LOW);
              analogWrite(enR, speedmothafucker);
              analogWrite(enL, speedmothafucker);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              delay(500);
              //analogWrite(motorR, 0);
              //analogWrite(motorL, 0);
              // now turn off motors
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, LOW);
              break;

            case 2:
              //fun beeps
              tone(beepPin, freq);
              delay(125);
              noTone(beepPin);
              tone(beepPin, freq * 2);
              delay(100);
              noTone(beepPin);
              tone(beepPin, freq / 2);
              delay(220);
              noTone(beepPin);
              tone(beepPin, freq);
              delay(125);
              noTone(beepPin);
              tone(beepPin, freq * 2);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              delay(100);
              noTone(beepPin);
              break;
            case 3:
              //one beep this time
              tone(beepPin, freq);
              delay(150);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              noTone(beepPin);
              break;
            case 4:
              //turns left?
              //analogWrite(motorR, speedmothafucker / 2);
              //analogWrite(motorL, 2 * speedmothafucker);
              digitalWrite(in1L, HIGH);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, HIGH);
              analogWrite(enR, speedmothafucker / 2);
              analogWrite(enL, speedmothafucker * 2);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              delay(500);
              //analogWrite(motorR, 0);
              //analogWrite(motorL, 0);
              // now turn off motors
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, LOW);
              break;
            case 5:
              //turns right?
              //analogWrite(motorR, 2 * speedmothafucker);
              //analogWrite(motorL, speedmothafucker / 2);
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, HIGH);
              digitalWrite(in3R, HIGH);
              digitalWrite(in4R, LOW);
              analogWrite(enR, speedmothafucker * 2);
              analogWrite(enL, speedmothafucker / 2);
              CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
              delay(500);
              //analogWrite(motorR, 0);
              //analogWrite(motorL, 0);
              // now turn off motors
              digitalWrite(in1L, LOW);
              digitalWrite(in2L, LOW);
              digitalWrite(in3R, LOW);
              digitalWrite(in4R, LOW);
              break;
            case 6:
              //This start the recording of the task
              if (task == LOW) {
                task = HIGH;
                steps = 0;
              }
              break;
            case 7:
              //this case stops recording the task
              if (task == HIGH) {
                task = LOW;
                File stepFile = SD.open("numSteps.txt", FILE_WRITE);
                stepString = String(steps);
                Serial.println(stepString);
                stepFile.println(stepString);
                stepFile.close();
              }
              break;
            case 8:
              //this case is for running the saved task, and is curretly under comstruction
              if (task == LOW) {
                activeTask == HIGH;
                File taskFile = SD.open("taska.txt", FILE_READ);
                while (taskFile.available()) {
                  if (proxr == LOW || proxl == LOW) {
                    digitalWrite(13, HIGH);
                    Serial.println("it did the thing");
                    tone(beepPin, 4200);
                    delay(256);
                    noTone(beepPin);
                    digitalWrite(13, LOW);

                  }
                  else {
                    Serial.println((taskFile.read()));
                    //because file I/O is a bitch the below case staments aren't the orignal int values but
                    //their ascii equivlents that will be read
                    switch (taskFile.read()) {
                      case 48:
                        //stop your life
                        // analogWrite(motorR, 0);
                        //analogWrite(motorL, 0);
                        noTone(beepPin);
                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, HIGH);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, HIGH);
                        analogWrite(enR, speedmothafucker);
                        analogWrite(enL, speedmothafucker);
                        delay(500);
                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, LOW);

                        break;
                      case 49:
                        //go go beep beep
                        // analogWrite(motorR, speedmothafucker);
                        //analogWrite(motorL, speedmothafucker);// will turn the LED on
                        digitalWrite(in1L, HIGH);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, HIGH);
                        digitalWrite(in4R, LOW);
                        analogWrite(enR, speedmothafucker);
                        analogWrite(enL, speedmothafucker );
                        delay(500);
                        //analogWrite(motorR, 0);
                        //analogWrite(motorL, 0);
                        // now turn off motors
                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, LOW);
                        break;
                      case 50:
                        //fun beeps
                        tone(beepPin, freq);
                        delay(125);
                        noTone(beepPin);
                        tone(beepPin, freq * 2);
                        delay(100);
                        noTone(beepPin);
                        tone(beepPin, freq / 2);
                        delay(220);
                        noTone(beepPin);
                        tone(beepPin, freq);
                        delay(125);
                        noTone(beepPin);
                        tone(beepPin, freq * 2);
                        delay(100);
                        noTone(beepPin);
                        break;
                      case 51:
                        //one beep this time
                        tone(beepPin, freq);
                        delay(150);
                        noTone(beepPin);
                        break;
                      case 52:
                        //turns left?
                        //analogWrite(motorR, speedmothafucker / 2);
                        //analogWrite(motorL, 2 * speedmothafucker);
                        digitalWrite(in1L, HIGH);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, HIGH);
                        analogWrite(enR, speedmothafucker / 2);
                        analogWrite(enL, speedmothafucker * 2);
                        delay(500);
                        //analogWrite(motorR, 0);
                        //analogWrite(motorL, 0);
                        // now turn off motors
                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, LOW);
                        break;
                      case 53:
                        //turns right?
                        //analogWrite(motorR, 2 * speedmothafucker);
                        //analogWrite(motorL, speedmothafucker / 2);

                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, HIGH);
                        digitalWrite(in3R, HIGH);
                        digitalWrite(in4R, LOW);
                        analogWrite(enR, speedmothafucker * 2);
                        analogWrite(enL, speedmothafucker / 2);
                        delay(500);
                        // now turn off motors
                        digitalWrite(in1L, LOW);
                        digitalWrite(in2L, LOW);
                        digitalWrite(in3R, LOW);
                        digitalWrite(in4R, LOW);
                        // analogWrite(motorR, 0);
                        //analogWrite(motorL, 0);
                        break;
                    }
                  }
                  activeTask = LOW;
                }
              }
              break;
            case 9:
              if (SD.exists("taska.txt")) {
                Serial.println(myData);
                SD.remove("taska.txt");
                Serial.println("that file is now gone");
              }
              break;
          }
          //here we get the IMU data
          //CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
          // convert from raw data to gravity and degrees/second units
          ax = convertRawAcceleration(aix);
          ay = convertRawAcceleration(aiy);
          az = convertRawAcceleration(aiz);
          gx = convertRawGyro(gix);
          gy = convertRawGyro(giy);
          gz = convertRawGyro(giz);

          // update the filter, which computes orientation
          filter.updateIMU(gx, gy, gz, ax, ay, az);

          // print the heading, pitch and roll to the data file
          roll = filter.getRoll();
          pitch = filter.getPitch();
          heading = filter.getYaw();
          File dataFile = SD.open("dataa.txt", FILE_WRITE);
          dataStringR = String(roll);
          dataStringP = String(pitch);
          dataStringH = String(heading);
          dataFile.print(" ");
          dataFile.print(roll);
          Serial.println(roll);
          dataFile.print(" ");
          dataFile.print(pitch);
          Serial.println(pitch);
          dataFile.print(" ");
          dataFile.print(heading);
          Serial.println(heading);
          dataFile.println(" ");
          dataFile.close();
          CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        }
      }
    }
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}



