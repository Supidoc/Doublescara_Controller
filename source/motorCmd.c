/************************************************************
 * @file    motorCmd.c
 * @brief   CLI commands for motor control functions
 * @details Implements CLI command handlers for motor movement, velocity/acceleration control,
 *          emergency stop, synchronized multi-motor movement, and freewheeling mode.
 *          Commands: mmove, mabs, mrev, mvel, macc, mstop, mestop, mclear, msync,
 *          mfree_on, mfree_off, mrun, mhold, mtest, mstatus.
 * @author  dg
 * @date    6 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motorCmd.h"
#include "motor.h"
#include "step_core.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "cli_utilities.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sync_wait.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define MCMD_COMMAND_TIMEOUT_MS 1000

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static BaseType_t cmd_motor_move_angle(char* pcWriteBuffer, size_t xWriteBufferLen,
                                       const char* pcCommandString);
static BaseType_t cmd_motor_move_absolute(char* pcWriteBuffer, size_t xWriteBufferLen,
                                          const char* pcCommandString);
static BaseType_t cmd_motor_move_revolutions(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString);
static BaseType_t cmd_motor_set_velocity(char* pcWriteBuffer, size_t xWriteBufferLen,
                                         const char* pcCommandString);
static BaseType_t cmd_motor_set_acceleration(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString);
static BaseType_t cmd_motor_stop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);
static BaseType_t cmd_motor_emergency_stop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                           const char* pcCommandString);
static BaseType_t cmd_motor_clear_emergency(char* pcWriteBuffer, size_t xWriteBufferLen,
                                            const char* pcCommandString);
static BaseType_t cmd_motor_get_angle(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);
static BaseType_t cmd_motor_get_state(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);
static BaseType_t cmd_motor_set_home(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString);
static BaseType_t cmd_motor_set_run_current(char* pcWriteBuffer, size_t xWriteBufferLen,
                                            const char* pcCommandString);
static BaseType_t cmd_motor_set_hold_current(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString);
static BaseType_t cmd_motor_synchronized_move(char* pcWriteBuffer, size_t xWriteBufferLen,
                                              const char* pcCommandString);
static BaseType_t cmd_motor_enable_freewheeling(char* pcWriteBuffer, size_t xWriteBufferLen,
                                                const char* pcCommandString);
static BaseType_t cmd_motor_disable_freewheeling(char* pcWriteBuffer, size_t xWriteBufferLen,
                                                 const char* pcCommandString);
static status_t   wait_for_mtr_cmd(CHD_CmdHandle_t cmdHandle, TickType_t deadline);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static const CLI_Command_Definition_t xMoveAngleCmd = {
    .pcCommand    = "mangle",
    .pcHelpString = "mangle -m <motor_id> -a <angle>: Move motor by relative angle in degrees\r\n",
    .pxCommandInterpreter        = cmd_motor_move_angle,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xMoveAbsoluteCmd = {
    .pcCommand    = "mabs",
    .pcHelpString = "mabs -m <motor_id> -a <angle>: Move motor to absolute angle in degrees\r\n",
    .pxCommandInterpreter        = cmd_motor_move_absolute,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xMoveRevolutionsCmd = {
    .pcCommand            = "mrev",
    .pcHelpString         = "mrev -m <motor_id> -r <revolutions>: Move motor by revolutions\r\n",
    .pxCommandInterpreter = cmd_motor_move_revolutions,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSetVelocityCmd = {
    .pcCommand            = "mvel",
    .pcHelpString         = "mvel -m <motor_id> -v <velocity>: Set motor velocity in deg/s\r\n",
    .pxCommandInterpreter = cmd_motor_set_velocity,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSetAccelerationCmd = {
    .pcCommand    = "macc",
    .pcHelpString = "macc -m <motor_id> -a <acceleration>: Set motor acceleration in deg/s²\r\n",
    .pxCommandInterpreter        = cmd_motor_set_acceleration,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xStopCmd = {
    .pcCommand    = "mstop",
    .pcHelpString = "mstop -m <motor_id> [-d]: Stop motor (optional -d for deceleration)\r\n",
    .pxCommandInterpreter        = cmd_motor_stop,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xEmergencyStopCmd = {
    .pcCommand                   = "mestop",
    .pcHelpString                = "mestop -m <motor_id>: Emergency stop for motor\r\n",
    .pxCommandInterpreter        = cmd_motor_emergency_stop,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xClearEmergencyCmd = {
    .pcCommand                   = "mclear",
    .pcHelpString                = "mclear: Clear emergency stop state\r\n",
    .pxCommandInterpreter        = cmd_motor_clear_emergency,
    .cExpectedNumberOfParameters = 0};

static const CLI_Command_Definition_t xGetAngleCmd = {
    .pcCommand                   = "mgetang",
    .pcHelpString                = "mgetang -m <motor_id>: Get current motor angle in degrees\r\n",
    .pxCommandInterpreter        = cmd_motor_get_angle,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xGetStateCmd = {
    .pcCommand                   = "mstate",
    .pcHelpString                = "mstate -m <motor_id>: Get motor movement state\r\n",
    .pxCommandInterpreter        = cmd_motor_get_state,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSetHomeCmd = {
    .pcCommand                   = "mhome",
    .pcHelpString                = "mhome -m <motor_id>: Set current position as home\r\n",
    .pxCommandInterpreter        = cmd_motor_set_home,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSetRunCurrentCmd = {
    .pcCommand            = "mirun",
    .pcHelpString         = "mirun -m <motor_id> -c <current>: Set run current in amperes\r\n",
    .pxCommandInterpreter = cmd_motor_set_run_current,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSetHoldCurrentCmd = {
    .pcCommand            = "mihold",
    .pcHelpString         = "mihold -m <motor_id> -c <current>: Set hold current in amperes\r\n",
    .pxCommandInterpreter = cmd_motor_set_hold_current,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xSynchronizedMoveCmd = {
    .pcCommand    = "msync",
    .pcHelpString = "msync -m1 <motor1> -a1 <angle1> -m2 <motor2> -a2 <angle2>: Synchronize "
                    "movement of 2 motors\r\n",
    .pxCommandInterpreter        = cmd_motor_synchronized_move,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xEnableFreewheelingCmd = {
    .pcCommand    = "mfree_on",
    .pcHelpString = "mfree_on -m <motor_id>: Enable freewheeling mode (deenergize coils)\r\n",
    .pxCommandInterpreter        = cmd_motor_enable_freewheeling,
    .cExpectedNumberOfParameters = -1};

static const CLI_Command_Definition_t xDisableFreewheelingCmd = {
    .pcCommand            = "mfree_off",
    .pcHelpString         = "mfree_off -m <motor_id>: Disable freewheeling and home motor\r\n",
    .pxCommandInterpreter = cmd_motor_disable_freewheeling,
    .cExpectedNumberOfParameters = -1};

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MCMD_init(void)
{
    return kStatus_Success;
}

void MCMD_task(void* pvParameters)
{
    (void)pvParameters; // Unused parameter

    LOG_INFO("Started MCMD Task");

    // Register all motor control commands
    CLI_register_command(&xMoveAngleCmd);
    CLI_register_command(&xMoveAbsoluteCmd);
    CLI_register_command(&xMoveRevolutionsCmd);
    CLI_register_command(&xSetVelocityCmd);
    CLI_register_command(&xSetAccelerationCmd);
    CLI_register_command(&xStopCmd);
    CLI_register_command(&xEmergencyStopCmd);
    CLI_register_command(&xClearEmergencyCmd);
    CLI_register_command(&xGetAngleCmd);
    CLI_register_command(&xGetStateCmd);
    CLI_register_command(&xSetHomeCmd);
    CLI_register_command(&xSetRunCurrentCmd);
    CLI_register_command(&xSetHoldCurrentCmd);
    CLI_register_command(&xSynchronizedMoveCmd);
    CLI_register_command(&xEnableFreewheelingCmd);
    CLI_register_command(&xDisableFreewheelingCmd);

    LOG_INFO("Motor commands registered");

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t wait_for_mtr_cmd(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}

static BaseType_t cmd_motor_move_angle(char* pcWriteBuffer, size_t xWriteBufferLen,
                                       const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            angle;

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get angle
    CLU_get_parameter_value_string("-a", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Angle required (-a)\r\n");
        return pdFALSE;
    }
    angle = strtod(pcParameter, NULL);

    if (MTR_move_angle_async(handle, angle, portMAX_DELAY, NULL) == kStatus_Success)
    {
        int32_t stepCount = MTR_angle_to_steps(handle, angle, ROUND_NEAREST);
        double  realAngle = MTR_steps_to_angle(handle, stepCount);
        snprintf(pcWriteBuffer, xWriteBufferLen, "Moving %.2f degrees\r\n", realAngle);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Move failed\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_move_absolute(char* pcWriteBuffer, size_t xWriteBufferLen,
                                          const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            angle;

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get angle
    CLU_get_parameter_value_string("-a", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Angle required (-a)\r\n");
        return pdFALSE;
    }
    angle = strtod(pcParameter, NULL);

    if (MTR_move_absolute_angle_async(handle, angle, portMAX_DELAY, NULL) == kStatus_Success)
    {
        int32_t stepCount = MTR_angle_to_steps(handle, angle, ROUND_NEAREST);
        double  realAngle = MTR_steps_to_angle(handle, stepCount);
        snprintf(pcWriteBuffer, xWriteBufferLen, "Moving to absolute angle %.2f degrees\r\n",
                 angle);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Move failed\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_move_revolutions(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            revolutions;

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get revolutions
    CLU_get_parameter_value_string("-r", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Revolutions required (-r)\r\n");
        return pdFALSE;
    }
    revolutions = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_move_revolutions_async(handle, revolutions, portMAX_DELAY, NULL) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Moving %.2f revolutions\r\n", revolutions);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Move failed\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_set_velocity(char* pcWriteBuffer, size_t xWriteBufferLen,
                                         const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            velocity;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get velocity
    CLU_get_parameter_value_string("-v", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Velocity required (-v)\r\n");
        return pdFALSE;
    }
    velocity = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_set_velocity_async(handle, velocity, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Velocity set to %.2f deg/s\r\n", velocity);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to set velocity\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_set_acceleration(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            acceleration;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get acceleration
    CLU_get_parameter_value_string("-a", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Acceleration required (-a)\r\n");
        return pdFALSE;
    }
    acceleration = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_set_acceleration_async(handle, acceleration, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Acceleration set to %.2f deg/s²\r\n",
                 acceleration);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to set acceleration\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_stop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    bool              decelerate = false;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Check for deceleration flag
    CLU_get_parameter_value_string("-d", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound)
    {
        decelerate = true;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_stop_async(handle, decelerate, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Motor stopped%s\r\n",
                 decelerate ? " with deceleration" : "");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to stop motor\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_emergency_stop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                           const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    if (MTR_emergency_stop(handle) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Emergency stop activated\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Emergency stop failed\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_clear_emergency(char* pcWriteBuffer, size_t xWriteBufferLen,
                                            const char* pcCommandString)
{
    (void)pcCommandString; // Unused parameter

    if (MTR_clear_emergency_stop() == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Emergency stop cleared\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to clear emergency stop\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_get_angle(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            angle;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_get_current_angle_async(handle, &angle, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Current angle: %.2f degrees\r\n", angle);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to get angle\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_get_state(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString)
{
    MTR_MotorHandle_t   handle = NULL;
    uint8_t             parameterFound;
    const char*         pcParameter;
    BaseType_t          parameterStringLength;
    STP_MovementState_t state;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_get_movement_state_async(handle, &state, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        const char* stateStr;
        switch (state)
        {
            case STP_MOVEMENT_STOPPED:
                stateStr = "STOPPED";
                break;
            case STP_MOVEMENT_ACCELERATING:
                stateStr = "ACCELERATING";
                break;
            case STP_MOVEMENT_CONST_VELOCITY:
                stateStr = "CONSTANT_VELOCITY";
                break;
            case STP_MOVEMENT_DECELRATING:
                stateStr = "DECELERATING";
                break;
            case STP_MOVEMENT_FINISHED:
                stateStr = "FINISHED";
                break;
            case STP_MOVEMENT_SUCCESSFUL:
                stateStr = "SUCCESSFUL";
                break;
            case STP_MOVEMENT_FAILED:
                stateStr = "FAILED";
                break;
            default:
                stateStr = "UNKNOWN";
                break;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen, "Movement state: %s\r\n", stateStr);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to get state\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_set_home(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_set_home_position_async(handle, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Home position set\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to set home position\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_set_run_current(char* pcWriteBuffer, size_t xWriteBufferLen,
                                            const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            current;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get current
    CLU_get_parameter_value_string("-c", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Current required (-c)\r\n");
        return pdFALSE;
    }
    current = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_set_run_current_async(handle, current, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Run current set to %.3f A\r\n", current);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to set run current\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_set_hold_current(char* pcWriteBuffer, size_t xWriteBufferLen,
                                             const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;
    double            current;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Get current
    CLU_get_parameter_value_string("-c", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Current required (-c)\r\n");
        return pdFALSE;
    }
    current = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_set_hold_current_async(handle, current, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Hold current set to %.3f A\r\n", current);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to set hold current\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_synchronized_move(char* pcWriteBuffer, size_t xWriteBufferLen,
                                              const char* pcCommandString)
{
    MTR_MotorHandle_t handles[2] = {NULL, NULL};
    double            angles[2]  = {0.0, 0.0};
    uint8_t           parameterFound;
    const char*       pcParameter;
    BaseType_t        parameterStringLength;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor 1 ID
    CLU_get_parameter_value_string("-m1", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 1 ID required (-m1)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handles[0]);
    if (handles[0] == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 1 not found\r\n");
        return pdFALSE;
    }

    // Get motor 1 angle
    CLU_get_parameter_value_string("-a1", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 1 angle required (-a1)\r\n");
        return pdFALSE;
    }
    angles[0] = strtod(pcParameter, NULL);

    // Get motor 2 ID
    CLU_get_parameter_value_string("-m2", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 2 ID required (-m2)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handles[1]);
    if (handles[1] == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 2 not found\r\n");
        return pdFALSE;
    }

    // Get motor 2 angle
    CLU_get_parameter_value_string("-a2", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor 2 angle required (-a2)\r\n");
        return pdFALSE;
    }
    angles[1] = strtod(pcParameter, NULL);

    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_synchronized_move_async(handles, angles, 2, deadline, &cmdHandle) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Synchronized move executed: Motor1=%.2f deg, Motor2=%.2f deg\r\n", angles[0],
                 angles[1]);
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Synchronized move failed\r\n");
    }

    return pdFALSE;
}

static BaseType_t cmd_motor_enable_freewheeling(char* pcWriteBuffer, size_t xWriteBufferLen,
                                                const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter = NULL;
    BaseType_t        parameterStringLength;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Call async function and wait
    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_enable_freewheeling_async(handle, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Freewheeling enabled\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to enable freewheeling\r\n");
    }
    return pdFALSE;
}

static BaseType_t cmd_motor_disable_freewheeling(char* pcWriteBuffer, size_t xWriteBufferLen,
                                                 const char* pcCommandString)
{
    MTR_MotorHandle_t handle = NULL;
    uint8_t           parameterFound;
    const char*       pcParameter = NULL;
    BaseType_t        parameterStringLength;

    TickType_t deadline = SYW_deadline_from_timeout_ms(MCMD_COMMAND_TIMEOUT_MS);

    // Get motor ID
    CLU_get_parameter_value_string("-m", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (!parameterFound || parameterStringLength == 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor ID required (-m)\r\n");
        return pdFALSE;
    }

    MTR_get_motor_by_label(pcParameter, &handle);
    if (handle == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Motor not found\r\n");
        return pdFALSE;
    }

    // Call async function and wait
    CHD_CmdHandle_t cmdHandle = NULL;
    if (MTR_disable_freewheeling_async(handle, deadline, &cmdHandle) == kStatus_Success &&
        wait_for_mtr_cmd(cmdHandle, deadline) == kStatus_Success)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Freewheeling disabled and motor homed\r\n");
    }
    else
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Failed to disable freewheeling\r\n");
    }
    return pdFALSE;
}
