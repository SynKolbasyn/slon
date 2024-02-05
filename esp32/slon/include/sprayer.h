#pragma once

#ifndef _SPRAYER_H_
#define _SPRAYER_H_


#include <ros.h>
#include <std_msgs/Bool.h>

#include "types.h"


using ros::Subscriber;


namespace sk {
namespace sprayer {


void setup_sprayer();
void loop_sprayer();
void sprayer_process(const std_msgs::Bool& state);


Subscriber<std_msgs::Bool> sub("sprayer_control", sprayer_process);

int sprayer_pin = 0;


void main_sprayer(void* pvParameters) {
  setup_sprayer();
  loop_sprayer();
}


void setup_sprayer() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.subscribe(sub);
  xSemaphoreGive(mutex);

  pinMode(sprayer_pin, OUTPUT);
}


void loop_sprayer() {
  while (true) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    vTaskDelay(500);
  }
}


void sprayer_process(const std_msgs::Bool& state) {
  digitalWrite(sprayer_pin, state.data);
}


} // namespace sprayer
} // namespace sk


#endif // !_SPRAYER_H_
