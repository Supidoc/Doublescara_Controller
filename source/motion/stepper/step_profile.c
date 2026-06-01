/************************************************************
 * @file    step_profile.c
 * @brief   Filedescription
 * @author  dg
 * @date    13 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <infrastructure/log.h>
#include "step_profile.h"
#include "stdio.h"
#include "math.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static inline void calculate_step_profile(STP_Handle_t handle);
static inline void calculate_delays(STP_Handle_t handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay);
static inline void calculate_acceleration_table_parameters(STP_Handle_t handle,
                                                           uint32_t*    interpFactor,
                                                           uint32_t*    tableSize);
static status_t create_accel_table(STP_Handle_t handle, uint32_t tableSize, uint16_t stepZeroDelay,
                                   uint16_t endVelocityDelay, int8_t* poolIndex);
static status_t calculate_acceleration_profile(STP_Handle_t handle, uint16_t* lookupTable,
                                               uint32_t numSteps, uint16_t stepZeroDelay,
                                               uint16_t endVelocityDelay);
static status_t allocate_accel_table(uint32_t tableSize, int8_t* poolIndex);
static void     free_accel_table(int8_t poolIndex);
static status_t set_direction_pin(STP_Handle_t handle);

/****************************
 *     Public Variables     *
 ****************************/
STP_AccelTablePoolItem_t accelTablePool[STP_ACCEL_TABLE_POOL_SIZE];

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t STPi_plan_motion_profile(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    calculate_step_profile(handle);

    uint16_t stepZeroDelay;
    uint16_t endVelocityDelay;
    calculate_delays(handle, &stepZeroDelay, &endVelocityDelay);

    uint32_t interpFactor;
    uint32_t tableSize;
    calculate_acceleration_table_parameters(handle, &interpFactor, &tableSize);

    int8_t poolIndex;
    if (create_accel_table(handle, tableSize, stepZeroDelay, endVelocityDelay, &poolIndex) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }

    // Store table information in movement handle
    handle->movementHandle.accelTablePoolIndex = poolIndex;
    handle->movementHandle.accelTableSize      = tableSize;
    handle->movementHandle.accelInterpFactor   = (uint16_t)interpFactor;

    handle->movementHandle.currStepCount = 0;
    handle->movementHandle.state         = STP_MOVEMENT_PLANNED;

    if (set_direction_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    static char logMsg[100];
    if (handle->movementHandle.isTrapezoidal)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Motion planned: trapezoidal (accel:%lu const:%lu decel:%lu)", handle->label,
                 handle->movementHandle.accelSteps, handle->movementHandle.endVelocitySteps,
                 handle->movementHandle.decelSteps);
    }
    else
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Motion planned: triangular (accel:%lu decel:%lu)",
                 handle->label, handle->movementHandle.accelSteps,
                 handle->movementHandle.decelSteps);
    }
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

status_t STPi_reset_movement_handle(STP_StepperMovement_t* handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    // Free lookup table back to pool if allocated
    if (handle->accelTablePoolIndex >= 0)
    {
        free_accel_table(handle->accelTablePoolIndex);
        handle->accelTablePoolIndex = -1;
    }

    handle->state              = STP_MOVEMENT_IDLE;
    handle->totalSteps         = 0;
    handle->accelSteps         = 0;
    handle->endVelocitySteps   = 0;
    handle->decelSteps         = 0;
    handle->direction          = STP_CLOCKWISE;
    handle->isTrapezoidal      = 0;
    handle->currStepCount      = 0;
    handle->phaseStepCount     = 0;
    handle->endVelocityDelay   = 0;
    handle->accelTableSize     = 0;
    handle->accelInterpFactor  = 1;
    handle->accelInterpCounter = 0;
    handle->accelTableIndex    = 0;
    handle->waitForStart       = 0;
    handle->cmdHandle          = NULL;
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static inline void calculate_step_profile(STP_Handle_t handle)
{
    uint32_t accelSteps =
        (uint32_t)((handle->endVelocity * handle->endVelocity) / (2 * handle->acceleration));

    if (accelSteps * 2 >= handle->movementHandle.totalSteps)
    {

        handle->movementHandle.isTrapezoidal = 0;

        handle->movementHandle.accelSteps = handle->movementHandle.totalSteps / 2;

        handle->movementHandle.decelSteps =
            handle->movementHandle.totalSteps - handle->movementHandle.accelSteps;

        handle->movementHandle.endVelocitySteps = 0;
    }
    else
    {

        handle->movementHandle.isTrapezoidal = 1;

        handle->movementHandle.accelSteps = accelSteps;
        handle->movementHandle.decelSteps = accelSteps;

        handle->movementHandle.endVelocitySteps =
            handle->movementHandle.totalSteps - 2 * accelSteps;
    }
}

static inline void calculate_delays(STP_Handle_t handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay)
{
    *stepZeroDelay = (uint16_t)(0.676 * STP_TIMER_FREQ_HZ * sqrt(2.0 / (handle->acceleration)));

    *endVelocityDelay = (uint16_t)(STP_TIMER_FREQ_HZ / (handle->endVelocity));

    handle->movementHandle.endVelocityDelay = *endVelocityDelay;
}

static inline void calculate_acceleration_table_parameters(STP_Handle_t handle,
                                                           uint32_t*    interpFactor,
                                                           uint32_t*    tableSize)
{
    *interpFactor = STP_ACCEL_TABLE_INTERP_FACTOR;

    if (*interpFactor < 1)
    {
        *interpFactor = 1;
    }

    if (handle->movementHandle.accelSteps > STP_MAX_ACCEL_TABLE_SIZE * (*interpFactor))
    {
        *interpFactor = (handle->movementHandle.accelSteps + STP_MAX_ACCEL_TABLE_SIZE - 1) /
                        STP_MAX_ACCEL_TABLE_SIZE;
    }

    *tableSize = (handle->movementHandle.accelSteps + *interpFactor - 1) / *interpFactor;

    if (*tableSize > STP_MAX_ACCEL_TABLE_SIZE)
    {
        *tableSize = STP_MAX_ACCEL_TABLE_SIZE;
    }
}

static status_t create_accel_table(STP_Handle_t handle, uint32_t tableSize, uint16_t stepZeroDelay,
                                   uint16_t endVelocityDelay, int8_t* poolIndex)
{

    if (handle == NULL || poolIndex == NULL)
    {
        return kStatus_Fail;
    }

    status_t status;

    status = allocate_accel_table(tableSize, poolIndex);

    if (status != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (tableSize > 0)
    {
        status = calculate_acceleration_profile(handle, accelTablePool[*poolIndex].table, tableSize,
                                                stepZeroDelay, endVelocityDelay);

        if (status != kStatus_Success)
        {
            free_accel_table(*poolIndex);
            return kStatus_Fail;
        }
    }

    return kStatus_Success;
}

static status_t allocate_accel_table(uint32_t tableSize, int8_t* poolIndex)
{
    if (poolIndex == NULL)
    {
        return kStatus_Fail;
    }

    for (int8_t i = 0; i < STP_ACCEL_TABLE_POOL_SIZE; i++)
    {
        if (accelTablePool[i].used == 0)
        {
            accelTablePool[i].used      = 1;
            accelTablePool[i].tableSize = tableSize;
            *poolIndex                  = i;
            return kStatus_Success;
        }
    }
    *poolIndex = -1;
    return kStatus_Fail; // No free table available
}

static void free_accel_table(int8_t poolIndex)
{
    if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE)
    {
        accelTablePool[poolIndex].used      = 0;
        accelTablePool[poolIndex].tableSize = 0;
    }
}

static status_t calculate_acceleration_profile(STP_Handle_t handle, uint16_t* lookupTable,
                                               uint32_t numSteps, uint16_t stepZeroDelay,
                                               uint16_t endVelocityDelay)
{
    if (lookupTable == NULL || numSteps == 0)
    {
        return kStatus_Fail;
    }

    // Start with initial delay, scaled up for fixed-point arithmetic
    uint32_t currentDelay = ((uint32_t)stepZeroDelay) << STP_DELAY_SCALE_BITS;
    uint32_t minDelay     = ((uint32_t)endVelocityDelay) << STP_DELAY_SCALE_BITS;

    // Calculate each step delay using David Austin formula
    for (uint32_t n = 1; n <= numSteps; n++)
    {
        // Store current delay (scaled down to uint16_t)
        lookupTable[n - 1] = (uint16_t)(currentDelay >> STP_DELAY_SCALE_BITS);

        // Calculate next delay: c_n = c_(n-1) - 2*c_(n-1) / (4*n + 1)
        // Using fixed-point arithmetic to maintain fractional precision
        for (uint32_t i = 0; i < handle->movementHandle.accelInterpFactor; i++)
        {
            uint32_t nextDelay = currentDelay - (2 * currentDelay / (4 * n + 1));

            // Clamp to minimum delay (end velocity)
            if (nextDelay < minDelay)
            {
                nextDelay = minDelay;
            }

            currentDelay = nextDelay;
        }
    }
    return kStatus_Success;
}

static status_t set_direction_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint8_t pinValue;
    if (handle->dirLogicHighClockwise)
    {
        pinValue = handle->movementHandle.direction == STP_CLOCKWISE;
    }
    else
    {
        pinValue = handle->movementHandle.direction == STP_COUNTERCLOCKWISE;
    }
    PCA_Write_Pin(handle->dirPort, handle->dirPin, pinValue);

    return kStatus_Success;
}
