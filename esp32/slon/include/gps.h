#include <Arduino.h>

#include <vector>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include "structs.h"

#pragma once

#ifndef _GPS_H_
#define _GPS_H_


#define ERROR_WITH_MODULE 0
#define HAVE_NOT_LOCATION 1
#define HAVE_LOCATION 2


static void smartDelay(unsigned long ms);


void set_status(short int status) {
  switch (status) {
  case ERROR_WITH_MODULE:
    /* TODO: Set led to red */
    
    break;

  case HAVE_NOT_LOCATION:
    /* TODO: Set led to yellow */
    break;

  case HAVE_LOCATION:
    /* TODO: Set led to green */
    break;
  }
}


void gps_process(TinyGPSPlus& gps, coordinates& coordinates) {
  bool isValid = gps.location.isValid();

  set_status(isValid?HAVE_LOCATION:HAVE_NOT_LOCATION);

  u64 distToNext = (u64)TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    coordinates.cords.at(coordinates.pos).f,
    coordinates.cords.at(coordinates.pos).s
  ) / 1000;

  double courseToNext = TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    coordinates.cords.at(coordinates.pos).f,
    coordinates.cords.at(coordinates.pos).s
  );

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    set_status(ERROR_WITH_MODULE);
  }
}


static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available()) gps.encode(ss.read());
  } while (millis() - start < ms);
}


#endif
