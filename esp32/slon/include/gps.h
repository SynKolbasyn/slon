#pragma once

#ifndef _GPS_H_
#define _GPS_H_


#include <array>

#include <Arduino.h>

#include <ros.h>
#include <slon/gps.h>

#include <TinyGPSPlus.h>

#include "types.h"
#include "structs.h"


using std::array;

using ros::Publisher;


namespace sk {
namespace gps {


slon::gps gps_data;

Publisher pub("gps_data", &gps_data);


enum GPS_status {
  ERROR_WITH_MODULE,
  HAVE_NOT_LOCATION,
  HAVE_LOCATION
};


void setup_gps();
void loop_gps();


void main_gps(void* pvParameters) {
  setup_gps();
  loop_gps();
}


void setup_gps() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.advertise(pub);
  xSemaphoreGive(mutex);

  Serial2.begin(9600);
}


void loop_gps() {
  TinyGPSPlus gps;

  GPS_status status = GPS_status::HAVE_NOT_LOCATION;
  Coordinates coordinates;

  while (true) {
    bool valid = gps.location.isValid();
    
    if (valid) status = GPS_status::HAVE_LOCATION;
    else status = GPS_status::HAVE_NOT_LOCATION;

    gps_data.state = false;
    gps_data.latitude = 0.0;
    gps_data.longitude = 0.0;

    // if (valid) Serial.printf("lat: %d | lon: %d\n", gps.location.lat(), gps.location.lng());
    while (Serial2.available()) gps.encode(Serial2.read());
    
    if (status == GPS_status::HAVE_LOCATION) {
      gps_data.state = true;
      gps_data.latitude = gps.location.lat();
      gps_data.longitude = gps.location.lng();
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    pub.publish(&gps_data);
    nh.spinOnce();
    xSemaphoreGive(mutex);

    vTaskDelay(1000);

    if (gps.charsProcessed() < 10) status = GPS_status::ERROR_WITH_MODULE;
  }
}


} // namespace gps
} // namespace sk


#endif
