#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <turbotrig/turbotrig.h>
#include <breezystm32/breezystm32.h>

#include "param.h"
#include "mixer.h"
#include "mux.h"
#include "estimator.h"

#include "controller.h"

int fsign(float y)
{
  return (0 < y) - (y < 0);
}

float fsat(float value, float max)
{
  if (abs(value) > abs(max))
  {
    value = max*sign(value);
  }
  return value;
}

int32_t sat(int32_t value, int32_t max)
{
  if (abs(value) > abs(max))
  {
    value = max*sign(value);
  }
  return value;
}

void run_controller(uint32_t now)
{
  control_t outgoing_command = rate_controller(_combined_control, now);
  _command.F = outgoing_command.F.value;
  _command.x = outgoing_command.x.value;
  _command.y = outgoing_command.y.value;
  _command.z = outgoing_command.z.value;
}

control_t altitude_controller(control_t altitude_command)
{
  // right now, this doesn't do anything, because I don't have an estimate of altitude
  control_t attitude_command = altitude_command;
  attitude_command.F.type = THROTTLE;
  return attitude_command;
}


control_t attitude_controller(control_t attitude_command, uint32_t now)
{
  static int32_t x_integrator = 0;
  static int32_t y_integrator = 0;
  static int32_t prev_time = 0;

  control_t rate_command = attitude_command;

  int32_t dt = (int32_t) now - prev_time;
  prev_time = now;

  if (attitude_command.x.type == ANGLE)
  {
    int32_t error = (attitude_command.x.value - _current_state.phi/1000);
    x_integrator += (error*dt)/1000;
    rate_command.x.value = (error*_params.values[PARAM_PID_ROLL_ANGLE_P])/1000
                           + (x_integrator/1000*_params.values[PARAM_PID_ROLL_ANGLE_I])/1000
                           - (_current_state.p/1000 *_params.values[PARAM_PID_ROLL_ANGLE_D]);
    // integrator anti-windup <-- Check this before fielding
    if (abs(rate_command.x.value) > _params.values[PARAM_MAX_ROLL_RATE])
    {
      // find the remaining space after the P and D terms to saturate
      int32_t space = _params.values[PARAM_MAX_ROLL_RATE]
                      - (error*_params.values[PARAM_PID_ROLL_ANGLE_P])/1000
                      + (_current_state.p/1000 *_params.values[PARAM_PID_ROLL_ANGLE_D]);;
      // Make the integrator fill that space
      x_integrator = (space*1000)/_params.values[PARAM_PID_ROLL_ANGLE_I];
      // Make sure the integrator never goes negative
      x_integrator = (x_integrator > 0) ? x_integrator : 0;
      // Saturate the signal
      rate_command.x.value = sat(rate_command.x.value, (int32_t)_params.values[PARAM_MAX_ROLL_RATE]);
    }
    rate_command.x.type = RATE;
  }

  if (attitude_command.y.type == ANGLE)
  {
    int32_t error = (attitude_command.y.value - _current_state.theta/1000);
    y_integrator += (error*dt)/1000;
    rate_command.y.value = (error*_params.values[PARAM_PID_PITCH_ANGLE_P])/1000
                           + (y_integrator*_params.values[PARAM_PID_PITCH_ANGLE_I])/1000
                           - (_current_state.q/1000 *_params.values[PARAM_PID_PITCH_ANGLE_D]);
    // integrator anti-windup
    if (abs(rate_command.y.value) > _params.values[PARAM_MAX_PITCH_RATE])
    {
      int32_t space = _params.values[PARAM_MAX_PITCH_RATE]
                      - (error*_params.values[PARAM_PID_PITCH_ANGLE_P])/1000
                      + (_current_state.p/1000 *_params.values[PARAM_PID_PITCH_ANGLE_D]);;
      y_integrator = (space*1000)/_params.values[PARAM_PID_PITCH_ANGLE_I];
      y_integrator = (y_integrator > 0) ? y_integrator : 0;
      rate_command.y.value = sat(rate_command.y.value, (int32_t)_params.values[PARAM_MAX_PITCH_RATE]);
    }
    rate_command.y.type = RATE;
  }

  return rate_command;
}


control_t rate_controller(control_t rate_command, uint32_t now)
{
  static float z_integrator = 0.0;
  static int32_t prev_time = 0;

  control_t motor_command = rate_command;

  int32_t dt = (int32_t) now - prev_time;
  prev_time = now;

  // Set values
  if (rate_command.x.active && rate_command.x.type == RATE)
  {
    float error = (float)(rate_command.x.value - _current_state.p); // mrad/s
    motor_command.x.value = (int32_t)fsat(error * (float)_params.values[PARAM_PID_ROLL_RATE_P]/1000.0f, (float)_params.values[PARAM_MAX_COMMAND]);
    motor_command.x.type = PASSTHROUGH;
  }

  if (rate_command.y.active && rate_command.y.type == RATE)
  {
    float error = (float)(rate_command.y.value - _current_state.q);
    motor_command.y.value = (int32_t)fsat(error * (float)_params.values[PARAM_PID_PITCH_RATE_P]/1000.0f, (float)_params.values[PARAM_MAX_COMMAND]);
    motor_command.y.type = PASSTHROUGH;
  }

  if (rate_command.z.active && rate_command.z.type == RATE)
  {
    bool used_anti_windup = false;
    float error = (float)(rate_command.z.value - _current_state.r); // urad
    z_integrator += 0;//(error*dt)/1000.0f; // urad
    motor_command.z.value = (int32_t)fsat(error * (float)_params.values[PARAM_PID_YAW_RATE_P]/1000.0f, (float)_params.values[PARAM_MAX_COMMAND]);
//                            + z_integrator/1000 * (float)_params.values[PARAM_PID_YAW_RATE_I]/1000.0f;
    // anti-windup
//    if (abs(motor_command.z.value) > _params.values[PARAM_MAX_COMMAND])
//    {
//      int32_t space = _params.values[PARAM_MAX_COMMAND]
//                      - (error*_params.values[PARAM_PID_YAW_RATE_P])/1000;
//      z_integrator = (space*1000)/_params.values[PARAM_PID_YAW_RATE_I];
//      z_integrator = (z_integrator > 0) ? z_integrator : 0;
//      motor_command.z.value = sat(motor_command.z.value, _params.values[PARAM_MAX_COMMAND]);
//      used_anti_windup = true;
//    }
    motor_command.z.type = PASSTHROUGH;
  }

  //  counter++;

  return motor_command;
}
#ifdef __cplusplus
}
#endif

