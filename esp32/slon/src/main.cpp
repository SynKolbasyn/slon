#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <PRIZM.h>
// 15; 400;

#include "proc.h"


EXPANSION prizm;

ros::NodeHandle  nh;

float pos = 0;


void ros_control(void* pvParameters);

void robot_control(void* pvParameters);

void message_cb(const std_msgs::Int32MultiArray& arr);


ros::Subscriber<std_msgs::Int32MultiArray> sub("main", &message_cb);


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(18, OUTPUT);
  prizm_setup(prizm);
  xTaskCreate(ros_control, "ros_control", 2048, NULL, 1, NULL);
  xTaskCreate(robot_control, "robot_control", 2048, NULL, 2, NULL);
}

void loop() {
  
}


void message_cb(const std_msgs::Int32MultiArray& arr) {
  pos = (arr.data[0] + arr.data[1]) / 2.0;
  digitalWrite(18, HIGH);
}


void ros_control(void* pvParameters) {
  while (true) {
    nh.spinOnce();
    vTaskDelay(100);
    digitalWrite(18, LOW);
    pos = 0;
  }
}


void robot_control(void* pvParameters) {
  while (true) {
    if (pos == 0) {
      robot_stop(prizm);
      continue;
    }
    if (pos > 340) robot_right(prizm, 20);
    if (pos < 300) robot_left(prizm, 20);
  }
}
