/************************************************************
 * @file    motor_homing.c
 * @brief   Filedescription
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_homing.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "motor_core.h"
#include "log.h"
#include "step_core.h"
#include "stdio.h"
#include "motor_convert.h"
#include "sync_wait.h"
#include "fsl_gpio.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

#define MHM_LEFT_ARM_LABEL "m_l_arm"
#define MHM_RIGHT_ARM_LABEL "m_r_arm"
#define MHM_LEFT_ARM_DIRECTION STP_COUNTERCLOCKWISE
#define MHM_RIGHT_ARM_DIRECTION STP_CLOCKWISE
#define MHM_HOME_MAX_ANGLE_DEG 180u

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static status_t init_homing(void);
static status_t auto_homing(MTR_MotorHandle_t handle, STP_Direction_t direction, uint16_t maxAngle,
                            TickType_t deadline);
static status_t wait_for_cmd_handle(CHD_CmdHandle_t cmdHandle, TickType_t deadline);
static status_t resolve_arm_handles(void);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/
static PORT_Type* l_arm_home_port = PORTD;
static uint8_t    l_arm_home_pin  = 5;
static PORT_Type* r_arm_home_port = PORTD;
static uint8_t    r_arm_home_pin  = 4;

static MTR_MotorHandle_t l_arm_handle = NULL;
static MTR_MotorHandle_t r_arm_handle = NULL;

static volatile uint8_t           homingSuccessful = 0;
static volatile MTR_MotorHandle_t homingHandle     = NULL;

static SemaphoreHandle_t homingMutex = NULL;
/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t MHM_init(void)
{
    status_t status = init_homing();
    if (status != kStatus_Success)
    {
        return status;
    }

    return resolve_arm_handles();
}

status_t MHM_home_left_arm(TickType_t deadline)
{
    if (MHM_init() != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (l_arm_handle == NULL)
    {
        return kStatus_Fail;
    }

    return auto_homing(l_arm_handle, MHM_LEFT_ARM_DIRECTION, MHM_HOME_MAX_ANGLE_DEG, deadline);
}

status_t MHM_home_right_arm(TickType_t deadline)
{
    if (MHM_init() != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (r_arm_handle == NULL)
    {
        return kStatus_Fail;
    }

    return auto_homing(r_arm_handle, MHM_RIGHT_ARM_DIRECTION, MHM_HOME_MAX_ANGLE_DEG, deadline);
}

void MHM_homing_arm_interrupt_handler(PORT_Type* port, uint32_t pin)
{
    MTR_MotorHandle_t currentHomingHandle = (MTR_MotorHandle_t)homingHandle;

    if (currentHomingHandle == NULL)
    {
        return;
    }

    if ((port == l_arm_home_port && pin == l_arm_home_pin) && currentHomingHandle == l_arm_handle)
    {
        if (STP_homing_stop(currentHomingHandle->stepperHandle) != kStatus_Success)
        {
            return;
        }
    }
    else if ((port == r_arm_home_port && pin == r_arm_home_pin) &&
             currentHomingHandle == r_arm_handle)
    {
        if (STP_homing_stop(currentHomingHandle->stepperHandle) != kStatus_Success)
        {
            return;
        }
    }
    else
    {
        return;
    }

    homingSuccessful = 1;
    homingHandle     = NULL;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t init_homing(void)
{
    if (homingMutex != NULL)
    {
        return kStatus_Success;
    }
    homingMutex = xSemaphoreCreateMutex();
    if (homingMutex == NULL)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t auto_homing(MTR_MotorHandle_t handle, STP_Direction_t direction, uint16_t maxAngle,
                            TickType_t deadline)
{
    status_t    status      = kStatus_Fail;
    BaseType_t  mutexLocked = pdFALSE;
    static char logMsg[100];

    if (emergencyStopFlag)
    {
        return kStatus_Fail;
    }
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    if (homingMutex == NULL && init_homing() != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (xSemaphoreTake(homingMutex, portMAX_DELAY) != pdTRUE)
    {
        return kStatus_Fail;
    }

    mutexLocked = pdTRUE;

    snprintf(logMsg, sizeof(logMsg), "[%s] Starting auto-homing procedure", handle->label);
    LOG_INFO(logMsg);

    homingSuccessful = 0;
    homingHandle     = handle;

    do
    {
        int32_t steps = -MTR_angle_to_steps(
            handle, (direction == STP_COUNTERCLOCKWISE ? maxAngle : -maxAngle), ROUND_UP);

        CHD_CmdHandle_t cmdHandle = NULL;
        if (STP_move_relative_async(handle->stepperHandle, steps, deadline, &cmdHandle) !=
            kStatus_Success)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Failed to queue auto-homing move command",
                     handle->label);
            LOG_ERROR(logMsg);
            break;
        }

        if (wait_for_cmd_handle(cmdHandle, deadline) == kStatus_Success)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Auto-homing reached max angle", handle->label);
            LOG_WARN(logMsg);
            break;
        }

        if (!homingSuccessful)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Auto-homing failed", handle->label);
            LOG_ERROR(logMsg);
            break;
        }

        if (MTR_set_home_position(handle) != kStatus_Success)
        {
            snprintf(logMsg, sizeof(logMsg), "[%s] Failed to set home position after auto-homing",
                     handle->label);
            LOG_ERROR(logMsg);
            break;
        }

        status = kStatus_Success;
    } while (0);

    homingHandle = NULL;

    if (mutexLocked == pdTRUE)
    {
        xSemaphoreGive(homingMutex);
    }

    if (status == kStatus_Success)
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Auto-homing procedure completed successfully",
                 handle->label);
        LOG_INFO(logMsg);
    }

    return status;
}

static status_t resolve_arm_handles(void)
{
    MTR_get_motor_by_label(MHM_LEFT_ARM_LABEL, &l_arm_handle);
    MTR_get_motor_by_label(MHM_RIGHT_ARM_LABEL, &r_arm_handle);

    if (l_arm_handle == NULL || r_arm_handle == NULL)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t wait_for_cmd_handle(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}

/* PORTD_IRQn interrupt handler */
void GPIOD_IRQHANDLER(void)
{
    /* Get pin flags */
    uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOD);

    /* Place your interrupt code here */

    /* Clear pin flags */
    GPIO_PortClearInterruptFlags(GPIOD, pin_flags);

    if (pin_flags & (1U << l_arm_home_pin))
    {
        MHM_homing_arm_interrupt_handler(l_arm_home_port, l_arm_home_pin);
    }
    if (pin_flags & (1U << r_arm_home_pin))
    {
        MHM_homing_arm_interrupt_handler(r_arm_home_port, r_arm_home_pin);
    }

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
   Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
