#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle  nh;

void ros_control(void* pvParameters) {
  while (true) {
    nh.spinOnce();
    vTaskDelay(100);
    digitalWrite(18, LOW);
  }
}

void message_cb(const std_msgs::Int32MultiArray& arr) {
  digitalWrite(18, HIGH);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("main", &message_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(18, OUTPUT);
  xTaskCreate(ros_control, "ros_control", 2048, NULL, 1, NULL);
}

void loop() {
  
}
