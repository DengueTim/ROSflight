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
#include "mode.h"

#include "controller.h"

#include "mavlink_log.h"
#include "mavlink_util.h"


void init_pid(pid_t* pid, float *kp, float *ki, float *kd, float *current_x, float *current_xdot, float *commanded_x, float *output, float max, float min)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->current_x = current_x;
  pid->current_xdot = current_xdot;
  pid->commanded_x = commanded_x;
  pid->output = output;
  pid->max = max;
  pid->min = min;
  pid->integrator = 0.0;
  pid->prev_time = micros()*1e-6;
  pid->differentiator = 0.0;
  pid->prev_x = 0.0;
  pid->tau = get_param_float(PARAM_PID_TAU);
}


void run_pid(pid_t *pid)
{
  // Time calculation
  float now = micros()*1e-6;
  float dt = now - pid->prev_time;
  pid->prev_time = now;
  if(dt > 0.010 || _armed_state == DISARMED)
  {
    // This means that this is a ''stale'' controller and needs to be reset.
    // This would happen if we have been operating in a different mode for a while
    // and will result in some enormous integrator.
    // Or, it means we are disarmed and shouldn't integrate
    // Setting dt for this loop will mean that the integrator and dirty derivative
    // doesn't do anything this time but will keep it from exploding.
    dt = 0.0;
    pid->differentiator = 0.0;
  }

  // Calculate Error (make sure to de-reference pointers)
  float error = (*pid->commanded_x) - (*pid->current_x);

  // Initialize Terms
  float p_term = error * (*pid->kp);
  float i_term = 0.0;
  float d_term = 0.0;

  // If there is a derivative term
  if(pid->kd != NULL)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    // (Be sure to de-refernce pointers)
    if(pid->current_xdot == NULL && dt > 0.0f)
    {
      pid->differentiator = (2.0f*pid->tau-dt)/(2.0f*pid->tau+dt)*pid->differentiator + 2.0f/(2.0f*pid->tau+dt)*((*pid->current_x) - pid->prev_x);
      pid->prev_x = *pid->current_x;
      d_term = (*pid->kd)*pid->differentiator;
    }
    else
    {
      d_term = (*pid->kd) * (*pid->current_xdot);
    }
  }

  // If there is an integrator, we are armed, and throttle is high
  /// TODO: better way to figure out if throttle is high
  if (pid->ki != NULL && _armed_state == ARMED && pwmRead(_params.values[PARAM_RC_F_CHANNEL] > 1200))
  {
    if ( (*pid->ki) > 0.0 ) {
      // integrate
      pid->integrator += error*dt;
      // calculate I term (be sure to de-reference pointer to gain)
      i_term = (*pid->ki) * pid->integrator;
    }
  }

  // sum three terms
  float u = p_term + i_term - d_term;

  // Integrator anti-windup
  float u_sat = (u > pid->max) ? pid->max : (u < pid->min) ? pid->min : u;
  if(u != u_sat && fabs(i_term) > fabs(u - p_term + d_term))
    pid->integrator = (u - p_term + d_term)/(*pid->ki);

  // Set output
  (*pid->output) = u_sat;

  return;
}


void init_controller()
{
  init_pid(&pid_roll,
           (float*)&_params.values[PARAM_PID_ROLL_ANGLE_P],
           (float*)&_params.values[PARAM_PID_ROLL_ANGLE_I],
           (float*)&_params.values[PARAM_PID_ROLL_ANGLE_D],
           &_current_state.phi,
           &_current_state.p,
           &_combined_control.x.value,
           &_command.x,
           _params.values[PARAM_MAX_COMMAND]/2.0f,
           -1.0f*_params.values[PARAM_MAX_COMMAND]/2.0f);

  init_pid(&pid_pitch,
           (float*)&_params.values[PARAM_PID_PITCH_ANGLE_P],
           (float*)&_params.values[PARAM_PID_PITCH_ANGLE_I],
           (float*)&_params.values[PARAM_PID_PITCH_ANGLE_D],
           &_current_state.theta,
           &_current_state.q,
           &_combined_control.y.value,
           &_command.y,
           _params.values[PARAM_MAX_COMMAND]/2.0f,
           -1.0f*_params.values[PARAM_MAX_COMMAND]/2.0f);

  init_pid(&pid_roll_rate,
           (float*)&_params.values[PARAM_PID_ROLL_RATE_P],
           (float*)&_params.values[PARAM_PID_ROLL_RATE_I],
           NULL,
           &_current_state.p,
           NULL,
           &_combined_control.x.value,
           &_command.x,
           _params.values[PARAM_MAX_COMMAND]/2.0f,
           -1.0f*_params.values[PARAM_MAX_COMMAND]/2.0f);

  init_pid(&pid_pitch_rate,
           (float*)&_params.values[PARAM_PID_PITCH_RATE_P],
           (float*)&_params.values[PARAM_PID_PITCH_RATE_I],
           NULL,
           &_current_state.q,
           NULL,
           &_combined_control.y.value,
           &_command.y,
           _params.values[PARAM_MAX_COMMAND]/2.0f,
           -1.0f*_params.values[PARAM_MAX_COMMAND]/2.0f);

  init_pid(&pid_yaw_rate,
           (float*)&_params.values[PARAM_PID_YAW_RATE_P],
           (float*)&_params.values[PARAM_PID_YAW_RATE_I],
           NULL,
           &_current_state.r,
           NULL,
           &_combined_control.z.value,
           &_command.z,
           _params.values[PARAM_MAX_COMMAND]/2.0f,
           -1.0f*_params.values[PARAM_MAX_COMMAND]/2.0f);

  init_pid(&pid_altitude,
           (float*)&_params.values[PARAM_PID_ALT_P],
           (float*)&_params.values[PARAM_PID_ALT_I],
           (float*)&_params.values[PARAM_PID_ALT_D],
           &_current_state.altitude,
           NULL,
           &_combined_control.F.value,
           &_command.F,
           _params.values[PARAM_MAX_COMMAND],
           0.0f);
}


void run_controller()
{
  // ROLL
  if(_combined_control.x.type == RATE)
    run_pid(&pid_roll_rate);
  else if(_combined_control.x.type == ANGLE)
    run_pid(&pid_roll);
  else // PASSTHROUGH
    _command.x = _combined_control.x.value;

  // PITCH
  if(_combined_control.y.type == RATE)
    run_pid(&pid_pitch_rate);
  else if(_combined_control.y.type == ANGLE)
    run_pid(&pid_pitch);
  else // PASSTHROUGH
    _command.y = _combined_control.y.value;

  // YAW
  if(_combined_control.z.type == RATE)
    run_pid(&pid_yaw_rate);
  else// PASSTHROUGH
    _command.z = _combined_control.z.value;

  // THROTTLE
  if(_combined_control.F.type == ALTITUDE)
    run_pid(&pid_altitude);
  else // PASSTHROUGH
    _command.F = _combined_control.F.value;

  static uint32_t counter = 0;
  if(counter > 100)
  {
    mavlink_send_named_command_struct("RC", _rc_control);
    mavlink_send_named_command_struct("offboard", _offboard_control);
    mavlink_send_named_command_struct("combined", _combined_control);
    mavlink_send_named_value_float("command_F", _command.F);
    mavlink_send_named_value_float("command_x", _command.x);
    mavlink_send_named_value_float("command_y", _command.y);
    mavlink_send_named_value_float("command_z", _command.z);
    counter = 0;
  }
  counter++;
}
