#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>
#include <turbotrig/turbovec.h>

#include "estimator.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mode.h"
#include "param.h"
#include "sensors.h"
#include "controller.h"
#include "mixer.h"
#include "rc.h"

extern void SetSysClock(bool overclock);

serialPort_t *Serial1;

int main(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();

  // Initialize Serial ports
  Serial1 = uartOpen(USART1, NULL, 921600, MODE_RXTX);

  // Perform Setup Operations
  setup();

  while (1)
  {
    // Main loop
    loop();
  }
}


void setup(void)
{
  // Make sure all the perhipherals are done booting up before starting
  delay(500);

  // Read EEPROM to get initial params
  init_params();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  //  // Initialize I2c
  i2cInit(I2CDEV_2);

  // Initialize PWM and RC
//  init_PWM();
//  init_rc();

  // Initialize MAVlink Communication
  init_mavlink();

  // Initialize Sensors
  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
//  init_mixing();

  // Initialize Estimator
//  init_estimator(false, true, true);
//  init_mode();
}


uint32_t counter = 0;
uint32_t average_time = 0;


void loop(void)
{
  uint32_t now = micros();
  /*********************/
  /***  Pre-Process ***/
  /*********************/
  // get loop time
//  static uint32_t prev_time;
//  static int32_t dt = 0;
//  uint32_t now = micros();

  /*********************/
  /***  Control Loop ***/
  /*********************/
  // update sensors - an internal timer runs this at a fixed rate
  update_sensors(now);


  /*********************/
  /***  Post-Process ***/
  /*********************/
  // internal timers figure out what to send
  mavlink_stream(now);

  // receive mavlink messages
  mavlink_receive();

  // update the armed_states, an internal timer runs this at a fixed rate
//  check_mode(now); // 0 us

  // get RC, an internal timer runs this every 20 ms (50 Hz)
//  receive_rc(now); // 1 us

  // update commands (internal logic tells whether or not we should do anything or not)
//  mux_inputs(); // 3 us
}
