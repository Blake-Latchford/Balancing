/**********************************************************************************************
   Arduino PID Library - Version 1.1.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

   This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "Arduino.h"

#include <math.h>
#include "PID.h"

#define D_TERM_FILTER_CONSTANT (0.25)

PID::PID(const double& kp, const double& ki, const double& kd )
  : kp(kp)
  , ki(ki)
  , kd(kd)
  , i_term(0.0)
  , lastInput(NAN)
{
  PID::SetOutputLimits(0, 255);
}


void PID::loop(const double& input, const double& setpoint, double& output)
{
  p_term = setpoint - input;
  i_term += (ki * p_term);
  i_term = clamp(i_term);

  if(isnan(lastInput)) lastInput = input;
  d_term =
    (d_term * D_TERM_FILTER_CONSTANT ) +
    (1 - D_TERM_FILTER_CONSTANT) * (input - lastInput);
  lastInput = input;

  output = clamp(kp * p_term + i_term - kd * d_term);
}


double PID::get_p_term()
{
  return p_term;
}

double PID::get_i_term()
{
  return i_term;
}

double PID::get_d_term()
{
  return d_term;
}

/* SetOutputLimits(...)****************************************************
       This function will be used far more often than SetInputLimits.  while
    the input to the controller will generally be in the 0-1023 range (which is
    the default already,)  the output will be a little different.  maybe they'll
    be doing a time window and will need 0-8000 or something.  or maybe they'll
    want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min > Max) {
    outMin = Max;
    outMax = Min;
  }
  else {
    outMin = Min;
    outMax = Max;
  }
}

void PID::Reset()
{
  i_term = 0.0;
  lastInput = NAN;
}

double PID::clamp(const double& input) {
  if (input > outMax) return outMax;
  else if (input < outMin) return outMin;
  else return input;
}

