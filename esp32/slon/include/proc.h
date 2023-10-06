#include <Arduino.h>

#include <Wire.h>


#pragma once

#ifndef _PROC_H_
#define _PROC_H_


void send_command(int id, int command) {
  Wire.beginTransmission(id);
  Wire.write(command);
  Wire.endTransmission();
  delay(15);
}


void send_command_four_byte(int id, int command, int byte1, int byte2, int byte3, int byte4) {
  Wire.beginTransmission(id);
  Wire.write(command);
  Wire.write(byte1);
  Wire.write(byte2);
  Wire.write(byte3);
  Wire.write(byte4);
  Wire.endTransmission();
  delay(15);
}

void set_motor_speeds(int address, int mSpeed1, int mSpeed2) {
  int lobyte1 = lowByte(mSpeed1);
  int hibyte1 = highByte(mSpeed1);

  int lobyte2 = lowByte(mSpeed2);
  int hibyte2 = highByte(mSpeed2);
  
  send_command_four_byte(address, 0x45, hibyte1, lobyte1, hibyte2, lobyte2);
}


void go(int lSpeed, int rSpeed) {
  set_motor_speeds(1, rSpeed, rSpeed);
  set_motor_speeds(2, rSpeed, rSpeed);
  set_motor_speeds(3, lSpeed, lSpeed);
  set_motor_speeds(4, lSpeed, lSpeed);
}


void motor_setup() {
  Wire.begin(21, 22);
  
  send_command(1, 0x27);
  send_command(2, 0x27);
  send_command(3, 0x27);
  send_command(4, 0x27);
  
  send_command(1, 0x25);
  send_command(2, 0x25);
  send_command(3, 0x25);
  send_command(4, 0x25);

  send_command(1, 0x4E);
  send_command(2, 0x4E);
  send_command(3, 0x4E);
  send_command(4, 0x4E);
}


void robot_left(int speed) {
  go(-100, 100);
}


void robot_right(int speed) {
  go(100, -100);
}


void robot_stop() {
  go(0, 0);
}


#endif