#ifndef TARGET_ANGLE_CONTROL_H
#define TARGET_ANGLE_CONTROL_H

class TargetAngleControl
{
public:
  TargetAngleControl(
    const double kp,
    const double equilibrium_guess_in_rad);
  void loop(
    const double& current_angle_in_rad,
    const double& current_drive_output_in_percent );

  double get_equilibrium_in_rad();
  double get_confidence_in_percent();
private:
  double equilibrium_in_rad;
  double confidence_in_percent;
  double kp;
};

#endif
