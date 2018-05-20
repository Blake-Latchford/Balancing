#include "drive_control.h"

#define LEFT_DIR_PIN 8
#define LEFT_SPEED_PIN 9
#define RIGHT_DIR_PIN 7
#define RIGHT_SPEED_PIN 6

DriveControl::DriveControl()
{}

void DriveControl::setup() {
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_SPEED_PIN, OUTPUT);
}

void DriveControl::loop(const double& left_drive_in_percent, const double& right_drive_in_percent) {
  drive(left_drive_in_percent, LEFT_DIR_PIN, LEFT_SPEED_PIN);
  drive(right_drive_in_percent, RIGHT_DIR_PIN, RIGHT_SPEED_PIN);
}

void DriveControl::drive(const double& drive_in_percent, const uint8_t dir_pin, const uint8_t speed_pin) {
  if ( drive_in_percent > 0 ) {
    digitalWrite( dir_pin, LOW );
  }
  else {
    digitalWrite( dir_pin, HIGH );
  }

  const int drive_pwm = map(1000*abs(drive_in_percent), 0, 1000, 30, 255);
  analogWrite(speed_pin, drive_pwm);
}

