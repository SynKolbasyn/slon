#include <Arduino.h>
#include <ros.h>

ros::NodeHandle nh;
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

#include "motors.h"
#include "compass.h"
#include "gps.h"
#include "bluetooth.h"
#include "sprayer.h"
#include "AIR_HORN.h"


using sk::motors::main_motors;
using sk::compass::main_compass;
using sk::gps::main_gps;
using sk::bluetooth::main_bluetooth;
using sk::sprayer::main_sprayer;
using sk::horn::main_horn;


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  xTaskCreate(main_motors, "motors", 2048, NULL, 1, NULL);
  xTaskCreate(main_compass, "compass", 2048, NULL, 1, NULL);
  xTaskCreate(main_gps, "gps", 2048, NULL, 1, NULL);
  xTaskCreate(main_bluetooth, "bluetooth", 4096, NULL, 1, NULL);
  xTaskCreate(main_sprayer, "sprayer", 2048, NULL, 1, NULL);
  xTaskCreate(main_horn, "horn", 2048, NULL, 1, NULL);
}


void loop() {
  vTaskDelay(1000000);
}
