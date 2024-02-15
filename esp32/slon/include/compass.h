#pragma once

#ifndef _COMPASS_H_
#define _COMPASS_H_


#include <Wire.h>

#include <ros.h>
#include <std_msgs/Float32.h>

#include <TroykaIMU.h>


using ros::Publisher;


namespace sk {
namespace compass {


std_msgs::Float32 compass_data;

Publisher pub("compass_data", &compass_data);

Madgwick filter;
Gyroscope gyroscope;
Accelerometer accelerometer;
Compass compass;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float yaw, pitch, roll;
float sampleRate = 100;

// http://wiki.amperka.ru/articles:troyka-magnetometer-compass-calibrate 
const float compassCalibrationBias[3] = { 567.893, -825.35, 1061.436 };
const float compassCalibrationMatrix[3][3] = {
  {1.909, 0.082, 0.004},
  {0.049, 1.942, -0.235},
  {-0.003, 0.008, 1.944}
};


void main_compass(void* pvParameters) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  nh.advertise(pub);
  xSemaphoreGive(mutex);

  gyroscope.begin();
  accelerometer.begin();
  compass.begin();
  filter.begin();
  compass.setCalibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);


  while (true) {
    unsigned long startMillis = millis();
    accelerometer.readAccelerationGXYZ(ax, ay, az);
    gyroscope.readRotationRadXYZ(gx, gy, gz);
    compass.readCalibrateMagneticGaussXYZ(mx, my, mz);
    filter.setFrequency(sampleRate);
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    yaw = filter.getYawDeg();
    pitch = filter.getPitchDeg();
    roll = filter.getRollDeg();
    
    // Serial.print("yaw: ");
    // Serial.print(yaw);
    // Serial.print("\t\t");
    // Serial.print("pitch: ");
    // Serial.print(pitch);
    // Serial.print("\t\t");
    // Serial.print("roll: ");
    // Serial.println(roll);
    
    float angle = 0.0;
    if (!isnan(yaw)) angle = yaw;
    compass_data.data = angle;
    xSemaphoreTake(mutex, portMAX_DELAY);
    pub.publish(&compass_data);
    nh.spinOnce();
    xSemaphoreGive(mutex);
    vTaskDelay(100);

    unsigned long deltaMillis = millis() - startMillis;
    sampleRate = 1000 / deltaMillis;
  }
}


} // namespace compass
} // namespace sk


#endif
