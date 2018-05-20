
#include "tilt_angle.h"
#include "PID.h"
#include "drive_control.h"
#include <SPI.h>
#include <SD.h>

#define K_P (1.0)

TiltAngle tilt_angle;
PID drive_control_pid( 0.0, 0.0, 0.0 );
DriveControl drive_control;
File data_log;

void interrupt_handler() {
    tilt_angle.interrupt_handler();
}

void setup() {
  Wire.begin();
  TWBR = 24;

  Serial.begin(115200);
  while(!Serial);

  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
  }
  else {
    data_log = SD.open("datalog.csv", FILE_WRITE);
    data_log.println("tilt_angle_in_rad,current_drive_output_in_percent");
  }

  tilt_angle.setup();
  drive_control.setup();

  attachInterrupt(
    digitalPinToInterrupt(TILT_ANGLE_INTERRUPT_PIN),
    interrupt_handler,
    RISING);

  drive_control_pid.SetOutputLimits(-1.0, 1.0);

}

void loop() {
  static bool new_tilt_angle_available = false;
  static double tilt_angle_in_rad = NAN;
  
  tilt_angle.loop(
    tilt_angle_in_rad,
    new_tilt_angle_available);

  if(new_tilt_angle_available) {
    new_angle_loop( tilt_angle_in_rad );
  }

}

void new_angle_loop( const double tilt_angle_in_rad )
{
  static double current_drive_output_in_percent = NAN;
  static double unflushed_log_lines = 0;
  
  drive_control_pid.loop(
    tilt_angle_in_rad,
    0.4,
    current_drive_output_in_percent);

  drive_control.loop(
    current_drive_output_in_percent,
    current_drive_output_in_percent);

  if(data_log) {
    data_log.print(tilt_angle_in_rad);
    data_log.print(",");
    data_log.println(current_drive_output_in_percent);
    unflushed_log_lines++;

    if(unflushed_log_lines > 50 ) {
      data_log.flush();
    }
  }
}
