#pragma once

#ifndef _PROC_BT_H_
#define _PROC_BT_H_


#include <string>

#include "BluetoothSerial.h"


BluetoothSerial bt;


void recv_bt(std::string& bt_data) {
  while (bt.available()) {
    bt_data += (char)bt.read();
    vTaskDelay(1);
  }
}


void send_bt(std::string& data) {
  uint8_t* buf = new uint8_t[data.size()];
  for (unsigned long int i = 0; i < data.size(); ++i) buf[i] = data[i];
  bt.write(buf, data.size());
  delete[] buf;
}


void setup_bt() {
  bt.begin("ESP32");
}


#endif
