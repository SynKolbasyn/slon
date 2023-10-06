#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include "proc.h"
#include "gps.h"
#include "structs.h"


static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);


ros::NodeHandle  nh;

float pos = 0;


void ros_control(void* pvParameters);

void robot_control(void* pvParameters);

void gps_control(void* pvParameters);

void message_cb(const std_msgs::Int32MultiArray& arr);


ros::Subscriber<std_msgs::Int32MultiArray> sub("main", &message_cb);


void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(16, OUTPUT);
  motor_setup();
  ss.begin(GPSBaud);
  xTaskCreate(ros_control, "ros_control", 2048, NULL, 1, NULL);
  xTaskCreate(robot_control, "robot_control", 2048, NULL, 2, NULL);
  xTaskCreate(gps_control, "gps_control", 2048, NULL, 2, NULL);
}

void loop() {
  
}


void message_cb(const std_msgs::Int32MultiArray& arr) {
  pos = (arr.data[0] + arr.data[1]) / 2.0;
  digitalWrite(16, HIGH);
}


void ros_control(void* pvParameters) {
  while (true) {
    nh.spinOnce();
    vTaskDelay(100);
    digitalWrite(16, LOW);
    pos = 0;
  }
}


void robot_control(void* pvParameters) {
  while (true) {
    if (pos == 0) {
      robot_stop();
      continue;
    }
    if (pos > 340) robot_right(20);
    if (pos < 300) robot_left(20);
  }
}


void gps_control(void* pvParameters) {
  pair<double, double> p1 = {.f = 1.23, .s = 2.34};
  pair<double, double> p2 = {.f = 1.23, .s = 2.34};
  std::vector<pair<double, double>> a = {p1, p2};
  coordinates c = {.cords = a, .pos = 0};
  while (true) {
    gps_process(gps, c);
  }
}
