/************************************************************
 * @file    motor.h
 * @brief   Filedescription
 * @author  dg
 * @date    2 Mar 2026
 ************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

/********************
 *     Includes    *
 ********************/
#include "step.h"
#include "tmc2209.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/***************************
 *     Public Typedefs     *
 ***************************/

typedef struct _MTR_MotorConfig
{
    STP_StepperConfig_t stepperHandle;
    TMC_Handle_t        tmcHandle;
    double              stepAngle;
    uint8_t             microstep;
    double              reductionFactor;
    char*               label;
} MTR_MotorConfig_t;

typedef struct _MTR_MotorHandle
{
    STP_StepperHandle_t stepperHandle;
    TMC_Handle_t        tmcHandle;
    double              stepAngle;
    uint8_t             microstep;
    double              reductionFactor;
    char*               label;
} MTR_MotorHandle_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

void MTR_task(void* pvParameters);

status_t MTR_init(void);

status_t MTR_init_motor(const MTR_MotorConfig_t* config, MTR_MotorHandle_t** handle);

void MTR_get_motor_by_label(const char* label, MTR_MotorHandle_t** handle);

status_t MTR_move_angle(MTR_MotorHandle_t* handle, double angle);

status_t MTR_move_absolute_angle(MTR_MotorHandle_t* handle, double angle);

status_t MTR_move_revolutions(MTR_MotorHandle_t* handle, double revolutions);

status_t MTR_set_velocity(MTR_MotorHandle_t* handle, double velocity_deg_per_sec);

status_t MTR_set_acceleration(MTR_MotorHandle_t* handle, double acceleration_deg_per_sec2);

status_t MTR_stop(MTR_MotorHandle_t* handle, bool decelerate);

status_t MTR_emergency_stop(MTR_MotorHandle_t* handle);

status_t MTR_clear_emergency_stop(void);

uint8_t MTR_is_emergency_stop_active(void);

status_t MTR_get_current_angle(MTR_MotorHandle_t* handle, double* angle);

status_t MTR_get_movement_state(MTR_MotorHandle_t* handle, STP_MovementState_t* state);

status_t MTR_wait_until_stopped(MTR_MotorHandle_t* handle, uint32_t timeout_ms);

status_t MTR_set_home_position(MTR_MotorHandle_t* handle);

status_t MTR_synchronized_move(MTR_MotorHandle_t** handles, double* angles, uint8_t count);

#endif /* MOTOR_H_ */
