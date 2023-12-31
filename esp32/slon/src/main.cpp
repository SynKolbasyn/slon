#include <Arduino.h>
#include <ros.h>

ros::NodeHandle nh;
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

#include "motors.h"
#include "compass.h"
#include "gps.h"
#include "bluetooth.h"


using sk::motors::main_motors;
using sk::compass::main_compass;
using sk::gps::main_gps;
using sk::bluetooth::main_bluetooth;


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  xTaskCreate(main_motors, "motors", 2048, NULL, 1, NULL);
  xTaskCreate(main_compass, "compass", 2048, NULL, 1, NULL);
  xTaskCreate(main_gps, "gps", 2048, NULL, 1, NULL);
  xTaskCreate(main_bluetooth, "bluetooth", 4096, NULL, 1, NULL);
}


void loop() {}
