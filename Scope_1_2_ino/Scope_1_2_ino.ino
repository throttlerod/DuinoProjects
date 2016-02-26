// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

/******************************************************************************
HMC6343_basics.ino
Simple example for using the HMC6343 3-axis compass library.
Jordan McConnell @ SparkFun Electronics
17 July 2014
https://github.com/sparkfun/HMC6343_Breakout

This example declares an SFE_HMC6343 object called compass. The object/sensor
is initialized and if the initialization fails, the sensor could not
be found and read from successfully.

Each time through the loop, heading values (heading, pitch, and roll) and
accelerometer values (accelX, accelY, accelZ) are read from the sensor. They
are then printed to the Serial Monitor at 115200 baud. The raw sensor values 
are printed as well as scaled values in readable units (degrees and g forces).

The HMC6343 breakout board needs to be powered with 3.3V and uses I2C to talk
to the microcontroller. If you're using a 5V microcontroller board, such as 
the standard Arduino UNO, you'll need a Logic Level Converter for the I2C lines,
such as SparkFun's BOB-12009.

Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5 & 1.5.2

Requires:
SFE_HMC6343_Library

This code is beerware.
Distributed as-is; no warranty is given. 
******************************************************************************/

/*
  LiquidCrystal Library - Serial Input
 
 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.
 
 This sketch displays text sent over the serial port 
 (e.g. from the Serial Monitor) on an attached LCD.
 
 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe 
 modified 22 Nov 2010
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/LiquidCrystalSerial
 */


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Libraries for I2C and the HMC6343 sensor
#include <Wire.h>
#include "SFE_HMC6343.h"
#include <TimeLib.h>
#include <LiquidCrystal.h>

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(8, 7);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;


Adafruit_GPS GPS(&mySerial);
SFE_HMC6343 compass; // Declare the sensor object

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false//true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Antenna Scope V1.2");
  delay(500);
  lcd.print("Antenna  Scope  V1.2");
  delay(10000);
  
  
  /*****/
  // Give the HMC6343 a half second to wake up
  //delay(500); 
  
  // Initialize the HMC6343 and verify its physical presence
  if (!compass.init())
  {
    Serial.println("Sensor Initialization Failed\n\r"); // Report failure, is the sensor wiring correct?
  }
  /*****/

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
   // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);
  // initialize the serial communications:
  //Serial.begin(115200);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
uint32_t timer1 = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour-7, DEC); Serial.print(':');
    printDigits(GPS.minute); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
 
    if (GPS.fix) {
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (MPH): "); Serial.println(GPS.speed*1.150779);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude (Feet): "); Serial.println(GPS.altitude*3.28084);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  /*****/
  // Read, calculate, and print the heading, pitch, and roll from the sensor
  compass.readHeading();
  printHeadingData();
  
  // Read, calculate, and print the acceleration on the x, y, and z axis of the sensor
  compass.readAccel();
  printAccelData();
  
  // Wait for two seconds
  delay(2000); // Minimum delay of 200ms (HMC6343 has 5Hz sensor reads/calculations)
  /*****/
  
  //LCD Print
  lcdprint();
  
  //voltRead();
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  lcd.setCursor(13,1);
  lcd.print("VCD");lcd.print(voltage);
  Serial.print("Volts"); Serial.print(voltage);
}


// Print both the raw values of the compass heading, pitch, and roll
// as well as calculate and print the compass values in degrees
// Sample Output:
// Heading Data (Raw value, in degrees):
// Heading: 3249  324.90°
// Pitch:   28    2.80°
// Roll:    24    2.40°
void printHeadingData()
{
  Serial.println("Heading Data (Raw value, in degrees):");
  Serial.print("Heading: ");
  Serial.print(compass.heading); Serial.print("  "); // Print raw heading value
  Serial.print((float) compass.heading/10.0);Serial.write(176);Serial.println(); // Print heading in degrees
  Serial.print("Pitch: ");
  Serial.print(compass.pitch); Serial.print("  ");
  Serial.print((float) compass.pitch/10.0);Serial.write(176);Serial.println();
  Serial.print("Roll: ");
  Serial.print(compass.roll); Serial.print("  ");
  Serial.print((float) compass.roll/10.0);Serial.write(176);Serial.println();
  Serial.println();
}

// Print both the raw values of the compass acceleration measured on each axis
// as well as calculate and print the accelerations in g forces
// Sample Output:
// Accelerometer Data (Raw value, in g forces):
// X: -52    -0.05g
// Y: -44    -0.04g
// Z: -1047  -1.02g
void printAccelData()
{
  Serial.println("Accelerometer Data (Raw value, in g forces):");
  Serial.print("X: ");
  Serial.print(compass.accelX); Serial.print("  "); // Print raw acceleration measurement on x axis
  Serial.print((float) compass.accelX/1024.0);Serial.println("g"); // Print x axis acceleration measurement in g forces
  Serial.print("Y: ");
  Serial.print(compass.accelY); Serial.print("  ");
  Serial.print((float) compass.accelY/1024.0);Serial.println("g");
  Serial.print("Z: ");
  Serial.print(compass.accelZ); Serial.print("  ");
  Serial.print((float) compass.accelZ/1024.0);Serial.println("g");
  Serial.println();
}

void lcdprint()
{
  //****LCD print****//
    
      lcd.setCursor(0,0);
      lcd.print("T:");
      printHour(GPS.hour); lcd.print(':');
      printDigitslcd(GPS.minute); lcd.print("   ");
      lcd.print("D:");
      lcd.print(GPS.month, DEC); lcd.print('/');
      lcd.print(GPS.day, DEC); lcd.print("/20");
      lcd.print(GPS.year, DEC);
      lcd.setCursor(0,1);
      lcd.print("Alt:"); lcd.print(GPS.altitude*3.28084);//lcd.print("FT");
      //lcd.print(" VDC"); lcd.print(voltage);
      lcd.setCursor(0,2);
      lcd.print("Head:");
      lcd.print(compass.heading/10.0);lcd.write(223);
      lcd.print("Tilt:"); lcd.print(compass.accelY);
      lcd.setCursor(0,3);
      lcd.print("Ang:"); lcd.print(GPS.angle);
      lcd.print(" Pitch"); lcd.print(compass.pitch);
  
}
         
void printDigitslcd(int digits) {
  if(digits < 10)
   lcd.print('0');
   lcd.print(digits);
  
}
 
 void printDigits(int digits) {
  if(digits < 10)
   Serial.print('0');
   Serial.print(digits);
  
} 

void printHour(int digit){
  if (GPS.month < 3 || GPS.month > 10)
  {lcd.print(GPS.hour-7);}
  else{ (GPS.hour-6);}
}


