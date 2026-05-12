/************************************************************
 * @file    step_timing.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "step_timing.h"
#include "step_internal.h"
#include "step_shared.h"
#include "stdio.h"
#include "step_profile.h"
#include "cmd_dispatch.h"
#include "log.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_Handle_t* handle);

/****************************
 *     Public Variables     *
 ****************************/
STP_Handle_t FTM3_ISR_handle_cache[8] = {NULL};

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/
uint8_t STPi_check_movement_completion(void)
{
    static char logMsg[80];
    uint8_t     nonIdleCount = 0;

    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used == 1 &&
            stpHandles[i].handle.movementHandle.state != STP_MOVEMENT_IDLE)
        {
            nonIdleCount++;
            if ((stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED ||
                 stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED))
            {

                if (stpHandles[i].handle.movementHandle.cmdHandle != NULL)
                {
                    CDP_notify_task_success(stpHandles[i].handle.movementHandle.cmdHandle);
                    CHD_remove_cmd_handle_ref(stpHandles[i].handle.movementHandle.cmdHandle);
                }

                if (stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED)
                {
                    snprintf(logMsg, sizeof(logMsg), "[%s] Movement finished",
                             stpHandles[i].handle.label);
                }
                else if (stpHandles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED)
                {
                    snprintf(logMsg, sizeof(logMsg), "[%s] Movement stopped",
                             stpHandles[i].handle.label);
                }
                LOG_DEBUG(logMsg);

                STPi_reset_movement_handle(&stpHandles[i].handle.movementHandle);
            }
        }
    }
    return nonIdleCount;
}

inline void STPi_update_isr_handle_cache(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        STP_Handle_t stepperHandle;
        get_handle_from_timer(FTM3, i, &stepperHandle);
        FTM3_ISR_handle_cache[i] = stepperHandle;
    }
}
/********************************************
 *     Private Function Implementations     *
 ********************************************/

void FTM3_IRQHandler(void)
{
    uint32_t intStatus;
    /* Reading all interrupt flags of status register */
    intStatus = FTM_GetStatusFlags(FTM3);
    FTM_ClearStatusFlags(FTM3, intStatus);

    traceISR_ENTER();

    for (uint8_t i = 0; i < 8; i++)
    {
        STP_Handle_t stepperHandle;

        if (intStatus & (0b1 << i) && (FTM3->CONTROLS[i].CnSC & FTM_CnSC_CHIE_MASK))
        {
            if (FTM3_ISR_handle_cache[i] == NULL)
            {
                continue;
            }
            stepperHandle = FTM3_ISR_handle_cache[i];

            stepperHandle->movementHandle.currStepCount++;

            stepperHandle->absolutePosition +=
                (stepperHandle->movementHandle.direction == STP_COUNTERCLOCKWISE) ? 1 : -1;

            uint32_t currentStepCount   = stepperHandle->movementHandle.currStepCount;
            uint32_t totalSteps         = stepperHandle->movementHandle.totalSteps;
            uint32_t accelSteps         = stepperHandle->movementHandle.accelSteps;
            uint32_t constVelocitySteps = stepperHandle->movementHandle.endVelocitySteps;

            // Toggle Pin because letting the time do it leads to inconsistencies in stepcount
            // because the start level for the timer output can't be defined

            GPIO_PortToggle(stepperHandle->stepGPIO, 0b1 << stepperHandle->stepPin);
            if (stepperHandle->stepPin == 2)
            {
                GPIO_PortToggle(GPIOB, 0b1 << 16);
            }
            else if (stepperHandle->stepPin == 9)
            {
                GPIO_PortToggle(GPIOB, 0b1 << 17);
            }

            if (currentStepCount >= totalSteps)
            {
                stepperHandle->ftmBase->CONTROLS[stepperHandle->ftmChannel].CnSC &=
                    ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);

                stepperHandle->movementHandle.state = STP_MOVEMENT_FINISHED;
                continue;
            }
            if (stepperHandle->movementHandle.state == STP_MOVEMENT_STARTED)
            {
                stepperHandle->movementHandle.state              = STP_MOVEMENT_ACCELERATING;
                stepperHandle->movementHandle.phaseStepCount     = 0;
                stepperHandle->movementHandle.accelInterpCounter = 0;
                stepperHandle->movementHandle.accelTableIndex    = 0;
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_ACCELERATING &&
                     currentStepCount > stepperHandle->movementHandle.accelSteps)
            {
                if (stepperHandle->movementHandle.isTrapezoidal)
                {
                    stepperHandle->movementHandle.state = STP_MOVEMENT_CONST_VELOCITY;
                }
                else
                {
                    stepperHandle->movementHandle.state              = STP_MOVEMENT_DECELRATING;
                    stepperHandle->movementHandle.phaseStepCount     = 0;
                    stepperHandle->movementHandle.accelInterpCounter = 0;
                    stepperHandle->movementHandle.accelTableIndex =
                        stepperHandle->movementHandle.accelTableSize - 1;
                }
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_CONST_VELOCITY &&
                     currentStepCount > accelSteps + constVelocitySteps)
            {
                stepperHandle->movementHandle.state              = STP_MOVEMENT_DECELRATING;
                stepperHandle->movementHandle.phaseStepCount     = 0;
                stepperHandle->movementHandle.accelInterpCounter = 0;
                stepperHandle->movementHandle.accelTableIndex =
                    stepperHandle->movementHandle.accelTableSize - 1;
            }

            // Increment phase step counter
            stepperHandle->movementHandle.phaseStepCount++;

            uint16_t actualDelay;
            int8_t   poolIndex = stepperHandle->movementHandle.accelTablePoolIndex;

            if (stepperHandle->movementHandle.state == STP_MOVEMENT_ACCELERATING)
            {
                // Use counter-based repeat factor (no division!)
                if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
                    stepperHandle->movementHandle.accelTableIndex <
                        accelTablePool[poolIndex].tableSize)
                {
                    actualDelay = accelTablePool[poolIndex]
                                      .table[stepperHandle->movementHandle.accelTableIndex];
                    // Increment counter and advance table index when counter reaches factor
                    stepperHandle->movementHandle.accelInterpCounter++;
                    if (stepperHandle->movementHandle.accelInterpCounter >=
                        stepperHandle->movementHandle.accelInterpFactor)
                    {
                        stepperHandle->movementHandle.accelInterpCounter = 0;
                        stepperHandle->movementHandle.accelTableIndex++;
                    }
                }
                else
                {
                    actualDelay = stepperHandle->movementHandle.endVelocityDelay;
                }
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_CONST_VELOCITY)
            {
                // Constant velocity: use endVelocityDelay directly
                actualDelay = stepperHandle->movementHandle.endVelocityDelay;
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_DECELRATING)
            {
                // Use counter-based repeat factor reading table backwards (no division!)
                if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
                    stepperHandle->movementHandle.accelTableIndex <
                        accelTablePool[poolIndex].tableSize)
                {
                    actualDelay = accelTablePool[poolIndex]
                                      .table[stepperHandle->movementHandle.accelTableIndex];
                    // Increment counter and decrement table index when counter reaches factor
                    stepperHandle->movementHandle.accelInterpCounter++;
                    if (stepperHandle->movementHandle.accelInterpCounter >=
                        stepperHandle->movementHandle.accelInterpFactor)
                    {
                        stepperHandle->movementHandle.accelInterpCounter = 0;
                        if (stepperHandle->movementHandle.accelTableIndex > 0)
                        {
                            stepperHandle->movementHandle.accelTableIndex--;
                        }
                    }
                }
                else
                {
                    actualDelay = stepperHandle->movementHandle.endVelocityDelay;
                }
            }
            else
            {
                actualDelay = stepperHandle->movementHandle.endVelocityDelay;
            }
            stepperHandle->ftmBase->CONTROLS[stepperHandle->ftmChannel].CnV += actualDelay;
        }
    }

    traceISR_EXIT();

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
   Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_Handle_t* handle)
{
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (stpHandles[i].used == 1 && stpHandles[i].handle.ftmBase == ftmBase &&
            stpHandles[i].handle.ftmChannel == ftmChannel)
        {
            *handle = &stpHandles[i].handle;
            return;
        }
    }
    *handle = NULL;
}
