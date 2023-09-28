#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle  nh;

bool led_state = false;

void message_cb(const std_msgs::Int32MultiArray& arr) {
  digitalWrite(18, HIGH);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("main", &message_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(18, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(100);
  digitalWrite(18, LOW);
}
