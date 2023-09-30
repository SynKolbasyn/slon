#include <Arduino.h>

#include <PRIZM.h>


void prizm_setup(EXPANSION& prizm) {
    prizm.controllerReset(1);
    prizm.controllerReset(2);
    prizm.controllerReset(3);
    prizm.controllerReset(4);
    
    delay(500);
    
    prizm.controllerEnable(1);
    prizm.controllerEnable(2);
    prizm.controllerEnable(3);
    prizm.controllerEnable(4);

    prizm.setMotorInvert(1, 0, 1);
    prizm.setMotorInvert(1, 1, 1);
    prizm.setMotorInvert(2, 0, 1);
    prizm.setMotorInvert(2, 1, 1);
}


void robot_left(EXPANSION& prizm, int speed) {
  prizm.setMotorPowers(1, -20, 20);
  prizm.setMotorPowers(2, -20, 20);
  prizm.setMotorPowers(3, 20, -20);
  prizm.setMotorPowers(4, 20, -20);
}


void robot_right(EXPANSION& prizm, int speed) {
  prizm.setMotorPowers(1, 20, -20);
  prizm.setMotorPowers(2, 20, -20);
  prizm.setMotorPowers(3, -20, 20);
  prizm.setMotorPowers(4, -20, 20);
}


void robot_stop(EXPANSION& prizm) {
  prizm.setMotorPowers(1, 125, 125);
  prizm.setMotorPowers(2, 125, 125);
  prizm.setMotorPowers(3, 125, 125);
  prizm.setMotorPowers(4, 125, 125);
}
