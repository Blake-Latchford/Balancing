#ifndef TILT_ANGLE_H
#define TILT_ANGLE_H

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define TILT_ANGLE_INTERRUPT_PIN 2

class TiltAngle {
public:
  TiltAngle();
  void setup();
  void loop( double& current_tilt_angle_in_rad, bool& new_tilt_anlge_available );
  
  void interrupt_handler();
private:
  enum class State {
    init,
    warm_up,
    running
  } state;

  void state_init();
  void state_warm_up();
  void state_running();

  double get_tilt_angle_in_rad();
  bool get_and_reset_tilt_angle_available();

  bool waiting_for_data();

  double current_tilt_angle_in_rad;
  bool new_tilt_anlge_available;
  
  uint16_t packet_size;
  uint8_t fifo_buffer[64];
  volatile bool mpu_data_ready = false;
};

#endif
