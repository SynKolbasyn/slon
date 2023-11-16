#pragma once

#ifndef _COMPASS_H_
#define _COMPASS_H_


#include <Wire.h>

#include <DFRobot_QMC5883.h>


DFRobot_QMC5883 compass(&Wire, /*I2C addr*/0x0D);


void calibrate();


void setup_compass() {
  Serial.begin(115200);

  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    vTaskDelay(500);
  }
}

void process_compass() {
  float declinationAngle = (10.0 + (91.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degress = ");
  Serial.println(mag.HeadingDegress);
  delay(100);
}


void calibrate() {
}

/*


#include <Wire.h>

void setup_compass() {
  Serial.begin (115200);  
  Wire.begin (21, 22);   // sda= GPIO_21 /scl= GPIO_22
}

void process_compass() {
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
  vTaskDelay(500);
}
*/

#endif
