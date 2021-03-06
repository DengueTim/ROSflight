#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "rc.h"
#include "param.h"
#include "mavlink_util.h"
#include "mux.h"

#include "mode.h"

#include "mavlink_log.h"


armed_state_t _armed_state;

void init_mode(void)
{
  _armed_state = DISARMED;
}

void arm(void)
{
  _armed_state = ARMED;
  LED1_ON;
}

void disarm(void)
{
  _armed_state = DISARMED;
  LED1_OFF;
}

/// TODO: Be able to tell if the RC has become disconnected during flight
bool check_failsafe(void)
{
  for(int8_t i = 0; i<_params.values[PARAM_RC_NUM_CHANNELS]; i++)
  {
    if(pwmRead(i) < 900 || pwmRead(i) > 2100)
    {
      if(_armed_state == ARMED || _armed_state == DISARMED)
      {
        _armed_state = FAILSAFE_DISARMED;
      }

      // blink LED
      static uint8_t count = 0;
      if(count > 25)
      {
        LED1_TOGGLE;
        count = 0;
      }
      count++;
      return true;
    }
  }

  // we got a valid RC measurement for all channels
  if(_armed_state == FAILSAFE_ARMED || _armed_state == FAILSAFE_DISARMED)
  {
    // return to appropriate mode
    _armed_state = (_armed_state == FAILSAFE_ARMED) ? ARMED : DISARMED;
  }
  return false;
}


bool check_mode(uint32_t now)
{
  static uint32_t prev_time = 0;
  static uint32_t time_sticks_have_been_in_arming_position = 0;

  // see it has been at least 20 ms
  uint32_t dt = now-prev_time;
  if (dt < 20000)
  {
    return false;
  }

  // if it has, then do stuff
  prev_time = now;

  // check for failsafe mode
  if(check_failsafe())
  {
    return true;
  }
  else
  {
    // check for arming switch
    if (_params.values[PARAM_ARM_STICKS])
    {
      if (_armed_state == DISARMED)
      {
        // if left stick is down and to the right
        if (pwmRead(_params.values[PARAM_RC_F_CHANNEL]) < _params.values[PARAM_RC_F_BOTTOM] + _params.values[PARAM_ARM_THRESHOLD]
            && pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) > (_params.values[PARAM_RC_Z_CENTER] + _params.values[PARAM_RC_Z_RANGE]/2)
            - _params.values[PARAM_ARM_THRESHOLD])
        {
          time_sticks_have_been_in_arming_position += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position = 0;
        }
        if (time_sticks_have_been_in_arming_position > 500000)
        {
          arm();
          time_sticks_have_been_in_arming_position = 0;
        }
      }
      else // _armed_state is ARMED
      {
        // if left stick is down and to the left
        if (pwmRead(_params.values[PARAM_RC_F_CHANNEL]) < _params.values[PARAM_RC_F_BOTTOM] +
            _params.values[PARAM_ARM_THRESHOLD]
            && pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) < (_params.values[PARAM_RC_Z_CENTER] - _params.values[PARAM_RC_Z_RANGE]/2)
            + _params.values[PARAM_ARM_THRESHOLD])
        {
          time_sticks_have_been_in_arming_position += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position = 0;
        }
        if (time_sticks_have_been_in_arming_position > 500000)
        {
          disarm();
          time_sticks_have_been_in_arming_position = 0;
        }
      }
    }
    else
    {
      if (rc_switch(_params.values[PARAM_ARM_CHANNEL]))
      {
        arm();
      }
      else
      {
        disarm();
      }
    }
  }
  return true;
}
