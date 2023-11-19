#pragma once

#ifndef _MOTORS_H_
#define _MOTORS_H_


#include <Arduino.h>
#include <Wire.h>

#include <ros.h>
#include <slon/motors.h>

#include "types.h"


namespace sk {
namespace motors {


using ros::Subscriber;


void setup_motors();
void loop_motors();
void motors_process(const slon::motors& speed);
void go(int lSpeed, int rSpeed);
void set_motor_speeds(int address, int mSpeed1, int mSpeed2);
void send_command_four_byte(int id, int command, int byte1, int byte2, int byte3, int byte4);
void send_command(int id, int command);


Subscriber<slon::motors> sub("motors_control", motors_process);


void main_motors(void* pvParameters) {
  setup_motors();
  loop_motors();
}


void setup_motors() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.subscribe(sub);
  xSemaphoreGive(mutex);

  Wire.begin(21, 22);
  
  for (i16 i = 1; i < 5; ++i) send_command(i, 0x27);
  for (i16 i = 1; i < 5; ++i) send_command(i, 0x25);
  for (i16 i = 1; i < 5; ++i) send_command(i, 0x4E);
}


void loop_motors() {
  while (true) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    vTaskDelay(100);
  }
}


void motors_process(const slon::motors& speed) {
  go(speed.l_speed, speed.r_speed);
}


void go(int lSpeed, int rSpeed) {
  set_motor_speeds(1, rSpeed, rSpeed);
  set_motor_speeds(2, rSpeed, rSpeed);
  set_motor_speeds(3, lSpeed, lSpeed);
  set_motor_speeds(4, lSpeed, lSpeed);
}


void set_motor_speeds(int address, int mSpeed1, int mSpeed2) {
  int lobyte1 = lowByte(mSpeed1);
  int hibyte1 = highByte(mSpeed1);

  int lobyte2 = lowByte(mSpeed2);
  int hibyte2 = highByte(mSpeed2);
  
  send_command_four_byte(address, 0x45, hibyte1, lobyte1, hibyte2, lobyte2);
}


void send_command_four_byte(int id, int command, int byte1, int byte2, int byte3, int byte4) {
  Wire.beginTransmission(id);
  Wire.write(command);
  Wire.write(byte1);
  Wire.write(byte2);
  Wire.write(byte3);
  Wire.write(byte4);
  Wire.endTransmission();
  vTaskDelay(15);
}


void send_command(int id, int command) {
  Wire.beginTransmission(id);
  Wire.write(command);
  Wire.endTransmission();
  vTaskDelay(15);
}


} // namespace motors
} // namespace sk


#endif
