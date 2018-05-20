#include "Arduino.h"
#include "target_angle_control.h"

#define CONFIDINCE_GAIN (0.1)

TargetAngleControl::TargetAngleControl(
    const double kp,
    const double equilibrium_guess_in_rad)
: equilibrium_in_rad(equilibrium_guess_in_rad)
, confidence_in_percent(0.0)
, kp(kp)
{}

void TargetAngleControl::loop(
  const double& current_angle_in_rad,
  const double& current_drive_output_in_percent )
{
  double angle_difference_in_rad = 
    abs( current_angle_in_rad - equilibrium_in_rad );
  
  double angle_difference_in_percent =  current_angle_in_rad / kp;

  if(angle_difference_in_percent > 1.0)
    angle_difference_in_percent = 1.0;

  //Cause confidence to slowly fall as the magnitude of the input rises.
  double confidence_modifier = -angle_difference_in_percent * current_drive_output_in_percent;
  //Cause the confidence to rapidly fall as the control loop exits the linear region.
  confidence_modifier += sq(abs(angle_difference_in_percent-current_drive_output_in_percent)-1);
  if(confidence_modifier < 0.0) {
    confidence_modifier = 0.0;
  }

  confidence_in_percent *= (1.0 - CONFIDINCE_GAIN);
  confidence_in_percent += CONFIDINCE_GAIN * confidence_modifier;

  double equilibrium_guess_in_rad = equilibrium_in_rad -
    (confidence_modifier * current_drive_output_in_percent / kp);
    
  equilibrium_in_rad *= confidence_in_percent;
  equilibrium_in_rad += (1 - confidence_in_percent) * equilibrium_guess_in_rad;
}

double TargetAngleControl::get_equilibrium_in_rad()
{
  return equilibrium_in_rad;
}

double TargetAngleControl::get_confidence_in_percent()
{
  return confidence_in_percent;
}


