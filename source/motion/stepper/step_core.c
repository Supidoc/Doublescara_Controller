/************************************************************
 * @file    step_core.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "step_core.h"
#include "step_internal.h"
#include "fsl_common.h"
#include "fsl_ftm.h"
#include "step_process.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t reset_absolute_position(STP_Handle_t handle);
static status_t set_absolute_position(STP_Handle_t handle, int32_t absoluteSteps);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

STP_HandlesArrayItem_t stpHandles[STP_MAX_HANDLE_COUNT];

CHD_CmdHandleImpl_t stpCmdHandles[STP_MAX_CMD_HANDLE_COUNT];

QueueHandle_t stpCmdQueue = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t STP_init()
{
    // Initialize acceleration table pool
    for (int8_t i = 0; i < STP_ACCEL_TABLE_POOL_SIZE; i++)
    {
        accelTablePool[i].used      = 0;
        accelTablePool[i].tableSize = 0;
    }

    stpCmdQueue = xQueueCreate(STP_CMD_QUEUE_SIZE, sizeof(STP_CmdQueueItem_t));
    if (stpCmdQueue == NULL)
    {
        return kStatus_Fail;
    }

    vQueueAddToRegistry(stpCmdQueue, "STP Command Queue");

    SIM->SCGC6 |= SIM_SCGC6_FTM3_MASK;

    NVIC_SetPriority(FTM3_IRQn, 2);
    NVIC_EnableIRQ(FTM3_IRQn);

    FTM3->MOD = 0xFFFF;
    FTM3->SC  = FTM_SC_CLKS(2) | FTM_SC_PS(0);

    CHD_init_cmd_handles(stpCmdHandles, STP_MAX_CMD_HANDLE_COUNT);

    return kStatus_Success;
}

void STP_task(void* pvParameters)
{
    LOG_INFO("Started Step Task");
    for (;;)
    {
        STPi_process();
    }
}

status_t STP_stop_async(STP_Handle_t handle, uint8_t doDeceleration, TickType_t deadline,
                        CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem       = {0};
    queueItem.handle                   = handle;
    queueItem.data.stop.doDeceleration = doDeceleration;
    queueItem.type                     = STP_CMD_STOP;
    queueItem.deadline                 = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_move_relative_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                 CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.handle             = handle;
    queueItem.data.move.steps    = steps;
    queueItem.type               = STP_CMD_MOVE;
    queueItem.deadline           = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_init_handle_async(STP_StepperConfig_t config, TickType_t deadline,
                               CHD_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.data.config.config = config;
    queueItem.type               = STP_CMD_INIT_HANDLE;
    queueItem.deadline           = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_handle_by_label(const char* label, STP_Handle_t* handle)
{
    if (label == NULL || handle == NULL)
    {
        return kStatus_Fail;
    }
    for (uint8_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used)
        {
            uint32_t handleLength = strlen(stpHandles[i].handle.label);

            if (strncmp(label, stpHandles[i].handle.label, handleLength) == 0)
            {
                *handle = &stpHandles[i].handle;
                return kStatus_Success;
            }
        }
    }
    return kStatus_Fail;
}

status_t STP_reset_absolute_position(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    return reset_absolute_position(handle);
}

status_t STP_move_relative_prepare_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                         CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.handle             = handle;
    queueItem.type               = STP_CMD_MOVE_PREPARE;
    queueItem.data.move.steps    = steps;
    queueItem.deadline           = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_trigger_prepared_moves_async(TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.type               = STP_CMD_TRIGGER_START;
    queueItem.deadline           = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_absolute_steps(STP_Handle_t handle, int32_t* absoluteSteps)
{
    if (handle == NULL || absoluteSteps == NULL)
    {
        return kStatus_Fail;
    }
    *absoluteSteps = handle->absolutePosition;
    return kStatus_Success;
}

status_t STP_set_absolute_steps(STP_Handle_t handle, int32_t absoluteSteps)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    return set_absolute_position(handle, absoluteSteps);
}

status_t STP_get_acceleration(STP_Handle_t handle, double* acceleration)
{
    if (handle == NULL || acceleration == NULL)
    {
        return kStatus_Fail;
    }
    *acceleration = handle->acceleration;
    return kStatus_Success;
}

status_t STP_get_end_velocity(STP_Handle_t handle, double* endVelocity)
{
    if (handle == NULL || endVelocity == NULL)
    {
        return kStatus_Fail;
    }
    *endVelocity = handle->endVelocity;
    return kStatus_Success;
}

status_t STP_get_movement_state(STP_Handle_t handle, STP_MovementState_t* state)
{
    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }
    *state = handle->movementHandle.state;
    return kStatus_Success;
}

status_t STP_set_acceleration_async(STP_Handle_t handle, double acceleration, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem                = {0};
    queueItem.handle                            = handle;
    queueItem.type                              = STP_CMD_SET_ACCELERATION;
    queueItem.data.setAcceleration.acceleration = acceleration;
    queueItem.deadline                          = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_set_end_velocity_async(STP_Handle_t handle, double endVelocity, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    if (endVelocity <= 0.0)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem              = {0};
    queueItem.handle                          = handle;
    queueItem.type                            = STP_CMD_SET_END_VELOCITY;
    queueItem.data.setEndVelocity.endVelocity = endVelocity;
    queueItem.deadline                        = deadline;

    return STPi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_default_config(STP_StepperConfig_t* config)
{
    if (config == NULL)
    {
        return kStatus_Fail;
    }

    config->acceleration          = 200;
    config->endVelocity           = 360;
    config->dirPin                = 0;
    config->dirPort               = PCA_PORT_0;
    config->ftmBase               = FTM0;
    config->ftmChannel            = kFTM_Chnl_0;
    config->stepGPIO              = GPIOA;
    config->stepPin               = 0;
    config->stepPort              = PORTA;
    config->label                 = "stepperMotor";
    config->dirLogicHighClockwise = 1;
    return kStatus_Success;
}

status_t STP_homing_stop(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    if (STPi_stop_steps_from_isr(handle, 0, NULL) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t reset_absolute_position(STP_Handle_t handle)
{
    taskENTER_CRITICAL();
    handle->absolutePosition = 0;
    taskEXIT_CRITICAL();
    return kStatus_Success;
}

static status_t set_absolute_position(STP_Handle_t handle, int32_t absoluteSteps)
{
    taskENTER_CRITICAL();
    handle->absolutePosition = absoluteSteps;
    taskEXIT_CRITICAL();
    return kStatus_Success;
}
