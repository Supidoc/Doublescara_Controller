/************************************************************
 * @file    motor_convert.c
 * @brief   Filedescription
 * @author  dg
 * @date    15 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_convert.h"
#include "motor_internal.h"
#include "limits.h"
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

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

double MTR_steps_to_angle(MTR_MotorHandle_t handle, int32_t steps)
{
    if (handle == NULL)
    {
        return 0.0;
    }

    if (handle->stepAngle <= 0.0 || handle->microstep == 0 || handle->reductionFactor <= 0.0)
    {
        return 0.0;
    }

    double angle = (double)steps * handle->stepAngle;
    angle        = angle / (double)handle->microstep;
    angle        = angle / handle->reductionFactor;
    return angle;
}

int32_t MTR_angle_to_steps(MTR_MotorHandle_t handle, double angle, MTR_roundingMethod_t method)
{
    if (handle == NULL)
    {
        return 0;
    }

    if (handle->stepAngle <= 0.0 || handle->microstep == 0 || handle->reductionFactor <= 0.0)
    {
        return 0;
    }

    double steps = angle * handle->reductionFactor;
    steps        = steps * (double)handle->microstep;
    steps        = steps / handle->stepAngle;

    if (steps < (double)INT32_MIN)
    {
        steps = (double)INT32_MIN;
    }
    if (steps > (double)INT32_MAX)
    {
        steps = (double)INT32_MAX;
    }

    int32_t result;
    switch (method)
    {
        case ROUND_DOWN:
            result = (int32_t)floor(steps);
            break;
        case ROUND_UP:
            result = (int32_t)ceil(steps);
            break;
        case ROUND_NEAREST:
            result = (int32_t)round(steps);
            break;
        default:
            result = (int32_t)round(steps);
            break;
    }
    return result;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
