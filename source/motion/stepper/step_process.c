/************************************************************
 * @file    step_process.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "step_process.h"
#include "step_internal.h"
#include "step_shared.h"
#include "cmd_dispatch.h"
#include "log.h"
#include "step_timing.h"
#include "step_profile.h"
#include "stdio.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static status_t STPi_init_handle(const STP_StepperConfig_t config, TickType_t deadline);
static status_t init_step_pin(STP_Handle_t handle);
static status_t init_dir_pin(STP_Handle_t handle);
static status_t start_steps(STP_Handle_t handle, CHD_CmdHandle_t cmdHandle);
static status_t start_prepared_moves_together(CHD_CmdHandle_t triggerCmdHandle,
                                              uint8_t*        anyStarted);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t STPi_send_cmd_async(STP_CmdQueueItem_t* queueItem, TickType_t deadline,
                             CHD_CmdHandle_t* cmdHandle)
{
    if (queueItem == NULL)
    {
        return kStatus_Fail;
    }
    CHD_CmdHandle_t internaleCmdHandle = NULL;

    status_t allocStatus =
        CHD_get_cmd_handle(&internaleCmdHandle, stpCmdHandles, STP_MAX_CMD_HANDLE_COUNT);
    if (allocStatus != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        CHD_add_cmd_handle_ref(internaleCmdHandle);
    }

    queueItem->cmdHandle = internaleCmdHandle;

    status_t sendStatus = CDP_send_cmd(stpCmdQueue, queueItem, deadline, internaleCmdHandle);
    if (sendStatus != kStatus_Success)
    {
        CHD_remove_cmd_handle_ref(internaleCmdHandle);
        if (cmdHandle != NULL)
        {
            *cmdHandle = NULL;
        }
        return kStatus_Fail;
    }
    if (cmdHandle != NULL)
    {
        *cmdHandle = internaleCmdHandle;
    }
    return kStatus_Success;
}

void STPi_process(void)
{
    uint8_t nonIdleMotors = STPi_check_movement_completion();

    STP_CmdQueueItem_t queueItem = {0};
    BaseType_t         status;
    TickType_t         delay = nonIdleMotors ? pdMS_TO_TICKS(2) : portMAX_DELAY;
    status                   = xQueueReceive(stpCmdQueue, &queueItem, delay);
    if (status != pdPASS)
    {
        return;
    }
    STPi_process_cmd(queueItem);
}

void STPi_process_cmd(STP_CmdQueueItem_t queueItem)
{
    static char logMsg[100];
    switch (queueItem.type)
    {
        case STP_CMD_MOVE:
        {
            LOG_DEBUG("Processing movement command");
            int32_t steps                               = queueItem.data.move.steps;
            queueItem.handle->movementHandle.totalSteps = (steps >= 0) ? steps : -steps;
            queueItem.handle->movementHandle.direction =
                (steps >= 0) ? STP_COUNTERCLOCKWISE : STP_CLOCKWISE;
            queueItem.handle->movementHandle.waitForStart = 0;
            if (STPi_plan_motion_profile(queueItem.handle) == kStatus_Success)
            {
                if (start_steps(queueItem.handle, queueItem.cmdHandle) != kStatus_Success)
                {
                    CDP_notify_task_failure(queueItem.cmdHandle);
                    CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
                    snprintf(logMsg, sizeof(logMsg), "[%s] Failed to start stepper execution",
                             queueItem.handle->label);
                    LOG_ERROR(logMsg);
                }
            }
            else
            {
                CDP_notify_task_failure(queueItem.cmdHandle);
                CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
                snprintf(logMsg, sizeof(logMsg), "[%s] Failed to plan motion profile for %lu steps",
                         queueItem.handle->label, queueItem.handle->movementHandle.totalSteps);
                LOG_ERROR(logMsg);
            }
            break;
        }
        case STP_CMD_MOVE_PREPARE:
        {
            LOG_DEBUG("Processing prepare movement command");
            int32_t steps                               = queueItem.data.move.steps;
            queueItem.handle->movementHandle.totalSteps = (steps >= 0) ? steps : -steps;
            queueItem.handle->movementHandle.direction =
                (steps >= 0) ? STP_COUNTERCLOCKWISE : STP_CLOCKWISE;
            queueItem.handle->movementHandle.waitForStart = 1;
            if (STPi_plan_motion_profile(queueItem.handle) == kStatus_Success)
            {
                CDP_notify_task_success(queueItem.cmdHandle);
            }
            else
            {
                CDP_notify_task_failure(queueItem.cmdHandle);
            }
            CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_TRIGGER_START:
        {
            LOG_DEBUG("Triggering synchronized start");

            uint8_t  anyStarted = 0;
            status_t startStatus;

            portENTER_CRITICAL();
            startStatus = start_prepared_moves_together(queueItem.cmdHandle, &anyStarted);
            portEXIT_CRITICAL();
            if (startStatus != kStatus_Success || !anyStarted)
            {
                CDP_notify_task_failure(queueItem.cmdHandle);
                CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            }

            break;
        }
        case STP_CMD_STOP:
        {
            LOG_DEBUG("Processing stop command");
            if (STPi_stop_steps(queueItem.handle, queueItem.data.stop.doDeceleration,
                                queueItem.cmdHandle) != kStatus_Success)
            {
                CDP_notify_task_failure(queueItem.cmdHandle);
                CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
                LOG_ERROR("Failed to stop steps");
            }
            break;
        }
        case STP_CMD_INIT_HANDLE:
        {
            LOG_DEBUG("Processing init handle command");
            if (STPi_init_handle(queueItem.data.config.config, queueItem.deadline) ==
                kStatus_Success)
            {
                CDP_notify_task_success(queueItem.cmdHandle);
            }
            else
            {
                CDP_notify_task_failure(queueItem.cmdHandle);
            }
            CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_SET_ACCELERATION:
        {
            LOG_DEBUG("Processing set acceleration command");
            queueItem.handle->acceleration = queueItem.data.setAcceleration.acceleration;
            CDP_notify_task_success(queueItem.cmdHandle);
            CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_SET_END_VELOCITY:
        {
            LOG_DEBUG("Processing set end velocity command");
            queueItem.handle->endVelocity = queueItem.data.setEndVelocity.endVelocity;
            CDP_notify_task_success(queueItem.cmdHandle);
            CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        default:
        {
            CDP_notify_task_failure(queueItem.cmdHandle);
            CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
            LOG_WARN("Received unknown command type");
            break;
        }
    }
}

status_t STPi_stop_steps(STP_Handle_t handle, uint8_t doDeceleration, CHD_CmdHandle_t cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    CHD_CmdHandle_t oldCmdHandle = NULL;

    STP_MovementState_t st = handle->movementHandle.state;
    if (st == STP_MOVEMENT_IDLE || st == STP_MOVEMENT_FINISHED || st == STP_MOVEMENT_STOPPED)
    {
        taskEXIT_CRITICAL();
        return kStatus_Fail;
    }

    oldCmdHandle = handle->movementHandle.cmdHandle;

    if (doDeceleration == 0)
    {
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &=
            ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~(FTM_CnSC_CHF_MASK);
        handle->movementHandle.state = STP_MOVEMENT_STOPPED;
    }
    else
    {
        if (st == STP_MOVEMENT_PLANNED)
        {
            // nothing running yet -> stop immediately
            handle->movementHandle.state = STP_MOVEMENT_STOPPED;
        }
        else if (st == STP_MOVEMENT_ACCELERATING)
        {
            handle->movementHandle.isTrapezoidal    = 0;
            handle->movementHandle.endVelocitySteps = 0;
            handle->movementHandle.accelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.decelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.totalSteps       = handle->movementHandle.currStepCount * 2;
        }
        else if (st == STP_MOVEMENT_CONST_VELOCITY)
        {
            handle->movementHandle.endVelocitySteps =
                handle->movementHandle.currStepCount - handle->movementHandle.accelSteps;
            handle->movementHandle.totalSteps = handle->movementHandle.accelSteps +
                                                handle->movementHandle.endVelocitySteps +
                                                handle->movementHandle.decelSteps;
        }
        // if already decelerating: keep as-is
    }

    handle->movementHandle.cmdHandle = cmdHandle;

    if (oldCmdHandle != NULL && oldCmdHandle != cmdHandle)
    {
        CDP_notify_task_failure(oldCmdHandle);
        CHD_remove_cmd_handle_ref(oldCmdHandle);
    }

    return kStatus_Success;
}

status_t STPi_stop_steps_from_isr(STP_Handle_t handle, uint8_t doDeceleration,
                                  CHD_CmdHandle_t cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    CHD_CmdHandle_t oldCmdHandle = NULL;

    STP_MovementState_t st = handle->movementHandle.state;
    if (st == STP_MOVEMENT_IDLE || st == STP_MOVEMENT_FINISHED || st == STP_MOVEMENT_STOPPED)
    {
        taskEXIT_CRITICAL();
        return kStatus_Fail;
    }

    oldCmdHandle = handle->movementHandle.cmdHandle;

    if (doDeceleration == 0)
    {
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &=
            ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~(FTM_CnSC_CHF_MASK);
        handle->movementHandle.state = STP_MOVEMENT_STOPPED;
    }
    else
    {
        if (st == STP_MOVEMENT_PLANNED)
        {
            // nothing running yet -> stop immediately
            handle->movementHandle.state = STP_MOVEMENT_STOPPED;
        }
        else if (st == STP_MOVEMENT_ACCELERATING)
        {
            handle->movementHandle.isTrapezoidal    = 0;
            handle->movementHandle.endVelocitySteps = 0;
            handle->movementHandle.accelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.decelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.totalSteps       = handle->movementHandle.currStepCount * 2;
        }
        else if (st == STP_MOVEMENT_CONST_VELOCITY)
        {
            handle->movementHandle.endVelocitySteps =
                handle->movementHandle.currStepCount - handle->movementHandle.accelSteps;
            handle->movementHandle.totalSteps = handle->movementHandle.accelSteps +
                                                handle->movementHandle.endVelocitySteps +
                                                handle->movementHandle.decelSteps;
        }
        // if already decelerating: keep as-is
    }

    handle->movementHandle.cmdHandle = cmdHandle;

    if (oldCmdHandle != NULL && oldCmdHandle != cmdHandle)
    {
        CDP_notify_task_failure_from_isr(oldCmdHandle);
        CHD_remove_cmd_handle_ref_from_isr(oldCmdHandle);
    }

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t STPi_init_handle(const STP_StepperConfig_t config, TickType_t deadline)
{
    static char logMsg[90];

    STP_Handle_t handle = NULL;
    for (uint8_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used == 0)
        {
            stpHandles[i].used = 1;
            handle             = &stpHandles[i].handle;
            break;
        }
    }

    if (handle == NULL)
    {
        LOG_ERROR("Failed to init stepper handle. Too many handles.");

        return kStatus_Fail;
    }

    handle->ftmBase    = config.ftmBase;
    handle->ftmChannel = config.ftmChannel;

    handle->stepPort = config.stepPort;
    handle->stepGPIO = config.stepGPIO;
    handle->stepPin  = config.stepPin;

    handle->dirPort               = config.dirPort;
    handle->dirPin                = config.dirPin;
    handle->dirLogicHighClockwise = config.dirLogicHighClockwise;

    handle->acceleration     = config.acceleration;
    handle->endVelocity      = config.endVelocity;
    handle->label            = config.label;
    handle->absolutePosition = 0;

    if (STPi_reset_movement_handle(&handle->movementHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (init_step_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (init_dir_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    STPi_update_isr_handle_cache();

    snprintf(logMsg, sizeof(logMsg),
             "[%s] Stepper initialized successfully (accel=%.1f step/s², vel=%.1f step/s)",
             handle->label, handle->acceleration, handle->endVelocity);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t start_steps(STP_Handle_t handle, CHD_CmdHandle_t cmdHandle)
{
    SEGGER_SYSVIEW_Print("Starting Steps");
    static char logMsg[100];

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    if (handle->movementHandle.state != STP_MOVEMENT_PLANNED)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Cannot start movement - not in PLANNED state (current: %d)", handle->label,
                 handle->movementHandle.state);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }

    handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~FTM_CnSC_CHF_MASK;

    // Use first value from lookup table
    uint16_t initialDelay;
    int8_t   poolIndex = handle->movementHandle.accelTablePoolIndex;

    if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
        accelTablePool[poolIndex].tableSize > 0)
    {
        initialDelay = accelTablePool[poolIndex].table[0];
    }
    else
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Lookup table retrieve failed for %lu steps at index %d (pool %d)",
                 handle->label, handle->movementHandle.totalSteps, 0, poolIndex);
        LOG_FATAL(logMsg);
        return kStatus_Fail;
    }
    handle->ftmBase->CONTROLS[handle->ftmChannel].CnV = handle->ftmBase->CNT + initialDelay;
    handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC |=
        FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK;

    handle->movementHandle.state     = STP_MOVEMENT_STARTED;
    handle->movementHandle.cmdHandle = cmdHandle;

    snprintf(logMsg, sizeof(logMsg), "[%s] Movement started: %lu steps", handle->label,
             handle->movementHandle.totalSteps);
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

static status_t init_step_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    gpio_pin_config_t gpioConfig;
    gpioConfig.pinDirection = kGPIO_DigitalOutput;
    gpioConfig.outputLogic  = GPIO_PinRead(handle->stepGPIO, handle->stepPin);

    GPIO_PinInit(handle->stepGPIO, handle->stepPin, &gpioConfig);
    PORT_SetPinMux(handle->stepPort, handle->stepPin, kPORT_MuxAlt1);

    return kStatus_Success;
}

static status_t init_dir_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    PCA_Pin_Config_t config;
    config.outputLevel  = PCA_PIN_LOW;
    config.pinDirection = PCA_PIN_OUTPUT;
    PCA_init_pin(handle->dirPort, handle->dirPin, config);

    return kStatus_Success;
}

static status_t start_prepared_moves_together(CHD_CmdHandle_t triggerCmdHandle, uint8_t* anyStarted)
{
    taskENTER_CRITICAL();
    if (anyStarted == NULL)
    {
        return kStatus_Fail;
    }

    STP_Handle_t    startHandles[STP_MAX_HANDLE_COUNT];
    CHD_CmdHandle_t startCmdHandles[STP_MAX_HANDLE_COUNT];
    uint16_t        initialDelays[STP_MAX_HANDLE_COUNT];
    uint8_t         startCount = 0;
    uint8_t         ownerSet   = 0;
    uint32_t        syncBaseCount;

    *anyStarted = 0;

    // Use the timer base of the first used handle (assume all on same timer)
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used &&
            stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_PLANNED &&
            stpHandles[i].handle.movementHandle.waitForStart)
        {
            syncBaseCount = stpHandles[i].handle.ftmBase->CNT;
            break;
        }
    }

    // Collect all prepared motors and their first-step delays
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used &&
            stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_PLANNED &&
            stpHandles[i].handle.movementHandle.waitForStart)
        {
            int8_t poolIndex = stpHandles[i].handle.movementHandle.accelTablePoolIndex;
            if (poolIndex < 0 || poolIndex >= STP_ACCEL_TABLE_POOL_SIZE ||
                accelTablePool[poolIndex].tableSize == 0)
            {
                stpHandles[i].handle.movementHandle.waitForStart = 0;
                continue;
            }
            stpHandles[i].handle.movementHandle.waitForStart = 0;
            startHandles[startCount]                         = &stpHandles[i].handle;
            initialDelays[startCount]                        = accelTablePool[poolIndex].table[0];
            startCmdHandles[startCount]                      = ownerSet ? NULL : triggerCmdHandle;
            if (!ownerSet)
            {
                ownerSet = 1;
            }
            startCount++;
        }
    }

    // Phase 1: write all compare values first
    for (uint8_t idx = 0; idx < startCount; idx++)
    {
        STP_Handle_t handle = startHandles[idx];
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~FTM_CnSC_CHF_MASK;
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnV = syncBaseCount + initialDelays[idx];
        if (handle->stepPin == 11)
        {
            GPIO_PortToggle(GPIOB, 0b1 << 16);
        }
        else if (handle->stepPin == 2)
        {
            GPIO_PortToggle(GPIOB, 0b1 << 17);
        }
    }

    // Phase 2: enable all channels
    for (uint8_t idx = 0; idx < startCount; idx++)
    {
        STP_Handle_t handle = startHandles[idx];
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC |=
            FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK;
        handle->movementHandle.state     = STP_MOVEMENT_STARTED;
        handle->movementHandle.cmdHandle = startCmdHandles[idx];

        if (handle->stepPin == 11)
        {
            GPIO_PortToggle(GPIOB, 0b1 << 16);
        }
        else if (handle->stepPin == 2)
        {
            GPIO_PortToggle(GPIOB, 0b1 << 17);
        }
    }
    taskEXIT_CRITICAL();

    *anyStarted = (startCount > 0) ? 1 : 0;
    return kStatus_Success;
}
