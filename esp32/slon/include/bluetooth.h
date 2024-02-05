#pragma once

#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_


#include <string>
#include <utility>

#include <BluetoothSerial.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <slon/bt.h>

#include "types.h"


using std::string;
using std::pair;

using ros::Subscriber;
using ros::Publisher;


namespace sk {
namespace bluetooth {


void setup_bluetooth();
void loop_bluetooth();
void message_process(const std_msgs::String& msg);
string recv_bt();
pair<string, i32> parse_bt(string& data);


BluetoothSerial bt;

slon::bt bluetooth_data;
Subscriber<std_msgs::String> sub("send_by_bluetooth", message_process);
Publisher pub("recieve_by_bluetooth", &bluetooth_data);


void main_bluetooth(void* pvParameters) {
  setup_bluetooth();
  loop_bluetooth();
}


void setup_bluetooth() {
  bt.begin("ESP32");

  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.advertise(pub);
  nh.subscribe(sub);
  xSemaphoreGive(mutex);
}


void loop_bluetooth() {
  while (true) {
    vTaskDelay(100);
    xSemaphoreTake(mutex, portMAX_DELAY);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    string data = recv_bt();
    if (data.empty()) continue;
    pair<string, i32> parsed_data = parse_bt(data);
    bluetooth_data.data = data.c_str();
    bluetooth_data.command = parsed_data.first.c_str();
    bluetooth_data.speed = parsed_data.second;
    xSemaphoreTake(mutex, portMAX_DELAY);
    pub.publish(&bluetooth_data);
    xSemaphoreGive(mutex);
  }
}


void message_process(const std_msgs::String& msg) {
  string data(msg.data);
  uint8_t buf[data.size()];
  for (u64 i = 0; i < data.size(); ++i) buf[i] = data[i];
  bt.write(buf, data.size());
}


string recv_bt() {
  string bt_data;
  while (bt.available()) {
    bt_data += (char)bt.read();
    vTaskDelay(1);
  }
  if (bt_data.find(";") == string::npos) return string();
  return bt_data;
}


pair<string, i32> parse_bt(string& data) {
  string command = data.substr(0, data.find(";"));
  i32 spd = stoi(data.substr(data.find(";") + 1, data.length()));
  return pair<string, i32> {command, spd};
}


} // namespace bluetooth
} // namespace sk


#endif
