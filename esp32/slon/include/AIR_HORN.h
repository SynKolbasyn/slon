#pragma once

#ifndef _AIR_HORN_H_
#define _AIR_HORN_H_


#include <ros.h>
#include <std_msgs/Bool.h>

#include "types.h"


namespace sk {
namespace horn {


void setup_horn();
void loop_horn();
void horn_process(const std_msgs::Bool& state);


int pin_horn = 33;
u64 prev_horn_time = 0;

ros::Subscriber<std_msgs::Bool> sub("horn_control", horn_process);


void main_horn(void* pvParametrs) {
  setup_horn();
  loop_horn();
}


void setup_horn() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.subscribe(sub);
  xSemaphoreGive(mutex);
  pinMode(pin_horn, OUTPUT);
  digitalWrite(pin_horn, LOW);
}


void loop_horn() {
  while (true) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    vTaskDelay(100);
  }
}


void horn_process(const std_msgs::Bool& state) {
  if (!state.data) {
    digitalWrite(pin_horn, LOW);
    return;
  }
  if (millis() < 15000 + prev_horn_time) {
    digitalWrite(pin_horn, LOW);
    return;
  }
  digitalWrite(pin_horn, HIGH);
  prev_horn_time = millis();
  while (millis() < 1000 + prev_horn_time) vTaskDelay(10);
  digitalWrite(pin_horn, LOW);
  prev_horn_time = millis();
}


} // namespace horn
} // namespace sk


#endif // !_AIR_HORN_H_
