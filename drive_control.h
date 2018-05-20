#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "Arduino.h"

class DriveControl {
public:
  DriveControl();

  void setup();
  void loop(const double& left_drive_in_percent, const double& right_drive_in_percent);
  
private:
  void drive(const double& drive_in_percent, const uint8_t dir_pin, const uint8_t speed_pin);
};

#endif
