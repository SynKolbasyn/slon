#include <Arduino.h>

#include <string>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include "proc.h"
#include "gps.h"
#include "structs.h"
#include "proc_bt.h"
#include "compass.h"


ros::NodeHandle  nh;

float pos = 0;

coordinates cords = {.cords = std::vector<pair<double, double>> {}, .pos = 0};


void ros_control(void* pvParameters);

void robot_control(void* pvParameters);

void gps_control(void* pvParameters);

void bt_send_control(void* pvParameters);

void bt_recv_control(void* pvParameters);

void compass_control(void* pvParametrs);

void message_cb(const std_msgs::Int32MultiArray& arr);


ros::Subscriber<std_msgs::Int32MultiArray> sub("main", &message_cb);


void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(16, OUTPUT);
  motor_setup();
  setup_bt();
  setup_gps();
  setup_compass();
  xTaskCreate(ros_control, "ros_control", 2048, NULL, 1, NULL);
  xTaskCreate(robot_control, "robot_control", 2048, NULL, 2, NULL);
  xTaskCreate(gps_control, "gps_control", 2048, NULL, 2, NULL);
  xTaskCreate(bt_send_control, "bt_send_control", 4096, NULL, 2, NULL);
  xTaskCreate(bt_recv_control, "bt_recv_control", 2048, NULL, 2, NULL);
  xTaskCreate(compass_control, "compass_control", 2048, NULL, 2, NULL);
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
  while (true) {
    gps_process(cords);
  }
}


void bt_send_control(void* pvParameters) {
  while (true) {
    std::string d(std::to_string((double)random() / random()) + ";" + std::to_string((double)random() / random()));
    send_bt(d);
    Serial.println(d.c_str());
    vTaskDelay(1000);
  }
}


void bt_recv_control(void* pvParameters) {
  while (true) {
    std::string d;
    recv_bt(d);
    if (d.size() != 0) Serial.println(d.c_str());
    if (d.compare("Save") == 0) {
      cords.cords.push_back(pair<double, double> {.f = (double)random() / random(), .s = (double)random() / random()});
      // cords.cords.push_back(pair<double, double> {.f = gps.location.lat(), .s = gps.location.lng()});
    }
  }
}


void compass_control(void* pvParametrs) {
  while(true) {
    process_compass();
    vTaskDelay(10);
  }
}
