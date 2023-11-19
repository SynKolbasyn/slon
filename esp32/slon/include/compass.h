#pragma once

#ifndef _COMPASS_H_
#define _COMPASS_H_


#include <Wire.h>

#include <ros.h>
#include <std_msgs/Float32.h>


// using ros::NodeHandle;
using ros::Publisher;


namespace sk {
namespace compass {


std_msgs::Float32 compass_data;

// NodeHandle nh;
Publisher pub("compass_data", &compass_data);


void main_compass(void* pvParameters) {
  // SemaphoreHandle_t mutex = (SemaphoreHandle_t) pvParameters;
  // nh.getHardware()->setBaud(115200);
  // nh.initNode();
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.advertise(pub);
  xSemaphoreGive(mutex);


  while (true) {
    float angle = 0.0;
    compass_data.data = angle;
    xSemaphoreTake(mutex, portMAX_DELAY);
    pub.publish(&compass_data);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    vTaskDelay(100);
  }
}


} // namespace compass
} // namespace sk


#endif
