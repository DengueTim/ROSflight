#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mavlink.h"

#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN

typedef enum
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  PARAM_BOARD_REVISION,
  PARAM_BAUD_RATE,

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  PARAM_SYSTEM_ID,
  PARAM_STREAM_HEARTBEAT_RATE,

  PARAM_STREAM_ATTITUDE_RATE,
  PARAM_STREAM_IMU_RATE,

  PARAM_CAMERA_TRIGGER_RATE,


  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  PARAM_INIT_TIME,
  PARAM_FILTER_KP,
  PARAM_FILTER_KI,
  PARAM_STREAM_ADJUSTED_GYRO,
  PARAM_GYRO_X_BIAS,
  PARAM_GYRO_Y_BIAS,
  PARAM_GYRO_Z_BIAS,
  PARAM_ACC_X_BIAS,
  PARAM_ACC_Y_BIAS,
  PARAM_ACC_Z_BIAS,
  PARAM_ACC_X_TEMP_COMP,
  PARAM_ACC_Y_TEMP_COMP,
  PARAM_ACC_Z_TEMP_COMP,

  // keep track of size of params array
  PARAMS_COUNT
} param_id_t;

typedef enum
{
  PARAM_TYPE_INT32,
  PARAM_TYPE_FLOAT,
  PARAM_TYPE_INVALID
} param_type_t;

// type definitions
typedef struct
{
  uint8_t version;
  uint16_t size;
  uint8_t magic_be;                       // magic number, should be 0xBE

  int32_t values[PARAMS_COUNT];
  char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
  param_type_t types[PARAMS_COUNT];

  uint8_t magic_ef;                       // magic number, should be 0xEF
  uint8_t chk;                            // XOR checksum
} params_t;

// global variable declarations
extern params_t _params;

// function declarations

/**
 * @brief Initialize parameter values
 */
void init_params(void);

/**
 * @brief Set all parameters to default values
 */
void set_param_defaults(void);

/**
 * @brief Read parameter values from non-volatile memory
 * @return True if successful, false otherwise
 */
bool read_params(void);

/**
 * @brief Write current parameter values to non-volatile memory
 * @return True if successful, false otherwise
 */
bool write_params(void);

/**
 * @brief Callback for executing actions that need to be taken when a parameter value changes
 * @param id The ID of the parameter that was changed
 */
void param_change_callback(param_id_t id);

/**
 * @brief Gets the id of a parameter from its name
 * @param name The name of the parameter
 * @return The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
 */
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

/**
 * @brief Sets the value of a parameter by ID and calls the parameter change callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_id(param_id_t id, int32_t value);

/**
 * @brief Sets the value of a parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value);

/**
 * @brief Sets the value of a floating point parameter by ID and calls the parameter callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return  True if a parameter was changed, false otherwise
 */
bool set_param_by_id_float(param_id_t id, float value);

/**
 * @brief Sets the value of a floating point parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value);

/**
 * @brief Get the value of a floating point parameter by id
 * @param id The ID of the parameter
 * @return The value of the parameter
 */
float get_param_float(param_id_t id);

#ifdef __cplusplus
}
#endif
