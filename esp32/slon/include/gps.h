#pragma once

#ifndef _GPS_H_
#define _GPS_H_


#include <Arduino.h>

#include <vector>

#include <TinyGPSPlus.h>

#include "structs.h"


#define ERROR_WITH_MODULE 0
#define HAVE_NOT_LOCATION 1
#define HAVE_LOCATION 2


TinyGPSPlus gps;


void setup_gps() {
  Serial2.begin(9600);
}


void set_status(i16 status) {
  switch (status) {
  case ERROR_WITH_MODULE:
    /* TODO: Set led to red */
    Serial.println("ERROR_WITH_MODULE");
    break;

  case HAVE_NOT_LOCATION:
    /* TODO: Set led to yellow */
    Serial.println("HAVE_NOT_LOCATION");
    break;

  case HAVE_LOCATION:
    /* TODO: Set led to green */
    Serial.println("HAVE_LOCATION");
    break;
  }
}


void gps_process(coordinates& cords) {
  bool is_valid = gps.location.isValid();

  set_status(is_valid?HAVE_LOCATION:HAVE_NOT_LOCATION);

  if (cords.cords.size() != 0) {
    u64 dist_to_next = (u64)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      cords.cords[cords.pos].f,
      cords.cords[cords.pos].s
    ) / 1000;

    double course_to_next = TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      cords.cords[cords.pos].f,
      cords.cords[cords.pos].s
    );
  }

  if (is_valid) Serial.printf("lat: %d | lon: %d\n", gps.location.lat(), gps.location.lng());

  while (Serial2.available()) gps.encode(Serial2.read());
  vTaskDelay(1000);

  if (gps.charsProcessed() < 10) set_status(ERROR_WITH_MODULE);
}


#endif
