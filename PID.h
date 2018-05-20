#ifndef PID_h
#define PID_h

class PID
{
  public:

    PID(const double& kp, const double& ki, const double& kd);

    void loop(const double& input, const double& setpoint, double& output);
    void SetOutputLimits(double Min, double Max);
    void SetSetpoint(double setpoint);

    void Reset();

    double get_p_term();
    double get_i_term();
    double get_d_term();
  private:
    double clamp(const double& input);

    const double kp;
    const double ki;
    const double kd;

    double p_term;
    double i_term;
    double d_term;
    
    double lastInput;

    double outMin, outMax;
};
#endif

