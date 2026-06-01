/************************************************************
 * @file    scara_kinematics.c
 * @brief   Five-bar SCARA kinematics core implementation
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "motor_core.h"
#include "sync_wait.h"
#include <math.h>
#include <motion/homing/motor_homing.h>
#include <motion/scara_kinematics/scara_kinematics.h>
#include <motion/scara_kinematics/scara_kinematics_internal.h>
#include <motion/scara_kinematics/scara_kinematics_process.h>
#include <string.h>

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static status_t SKi_apply_transform(const SK_Point_t* topLevelPoint, SK_Point_t* robotPoint);
static status_t SKi_inverse_transform(const SK_Point_t* robotPoint, SK_Point_t* topLevelPoint);
static bool     SKi_point_in_rect(const SK_WorkspaceRect_t* rect, const SK_Point_t* point);
static status_t SKi_workspace_allowed(const SK_Point_t* point, bool* allowed);

/****************************
 *     Public Variables     *
 ****************************/

SK_State_t skState = {
    .pose            = {.x = 30.0, .y = 220.0},
    .transform       = {1.0, 1.0, 0.0, 0.0},
    .workspaceLimits = {.rectangles = {{0}}, .rectangleCount = 0U, .enabled = false},
    .side            = SK_SIDE_MAIN,
    .calibrated      = false,
};

MTR_MotorHandle_t skLeftMotorHandle  = NULL;
MTR_MotorHandle_t skRightMotorHandle = NULL;
MTR_MotorHandle_t skZetaMotorHandle  = NULL;

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t SK_init(void)
{
    if (SKi_init_queue() != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}
status_t SK_init_motor_handles(MTR_MotorHandle_t leftMotorHandle,
                               MTR_MotorHandle_t rightMotorHandle,
                               MTR_MotorHandle_t zetaMotorHandle)
{
    if (leftMotorHandle == NULL || rightMotorHandle == NULL || zetaMotorHandle == NULL)
    {
        return kStatus_Fail;
    }

    skLeftMotorHandle  = leftMotorHandle;
    skRightMotorHandle = rightMotorHandle;
    skZetaMotorHandle  = zetaMotorHandle;

    return kStatus_Success;
}

void SK_task(void* pvParameters)
{
    SKi_task(pvParameters);
}

status_t SK_set_transform_async(SK_Transform_t transform, TickType_t deadline,
                                CHD_CmdHandle_t* cmdHandle)
{

    SK_CmdQueueItem_t queueItem = {.type                        = SK_CMD_SET_TRANSFORM,
                                   .deadline                    = deadline,
                                   .cmdHandle                   = NULL,
                                   .data.setTransform.transform = transform};

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_set_workspace_limits_async(SK_WorkspaceLimits_t limits, TickType_t deadline,
                                       CHD_CmdHandle_t* cmdHandle)
{

    SK_CmdQueueItem_t queueItem = {.type                           = SK_CMD_SET_WORKSPACE_LIMITS,
                                   .deadline                       = deadline,
                                   .cmdHandle                      = NULL,
                                   .data.setWorkspaceLimits.limits = limits};

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_enable_workspace_limits_async(bool enabled, TickType_t deadline,
                                          CHD_CmdHandle_t* cmdHandle)
{
    SK_CmdQueueItem_t queueItem = {.type      = SK_CMD_ENABLE_WORKSPACE_LIMITS,
                                   .deadline  = deadline,
                                   .cmdHandle = NULL,
                                   .data.enableWorkspaceLimits.enabled = enabled};

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_set_origin_offset_async(SK_Point_t offset, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle)
{
    SK_CmdQueueItem_t queueItem = {
        .type                        = SK_CMD_SET_ORIGIN_OFFSET,
        .deadline                    = deadline,
        .cmdHandle                   = NULL,
        .data.setOriginOffset.offset = offset,
    };

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_change_side_async(TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    SK_CmdQueueItem_t queueItem = {
        .type      = SK_CMD_CHANGE_SIDE,
        .deadline  = deadline,
        .cmdHandle = NULL,
    };

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_is_point_allowed(SK_Point_t point, bool* allowed)
{
    if (allowed == NULL)
    {
        return kStatus_Fail;
    }

    return SKi_workspace_allowed(&point, allowed);
}

status_t SK_inverse_kinematics(SK_Point_t point, double* theta1, double* theta2, double* zetaE)
{
    SK_Point_t robotPoint;
    double     d1E;
    double     d2E;
    double     q1;
    double     q2;
    double     cosU1;
    double     cosU2;

    const double c  = SK_BASE_SPAN_MM;
    const double a1 = SK_LEFT_ROOT_ARM_LENGTH_MM;
    const double a2 = SK_RIGHT_ROOT_ARM_LENGTH_MM;
    const double a3 = SK_LEFT_FAR_ARM_LENGTH_MM;
    const double a4 = SK_RIGHT_FAR_ARM_LENGTH_MM;

    if (SKi_apply_transform(&point, &robotPoint) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    bool allowed = true;
    if (SKi_workspace_allowed(&point, &allowed) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (!allowed)
    {
        return kStatus_Fail;
    }

    d1E = hypot(robotPoint.x, robotPoint.y);
    d2E = hypot(robotPoint.x - c, robotPoint.y);

    if ((d1E <= 0.0) || (d2E <= 0.0))
    {
        return kStatus_Fail;
    }

    //    cosQ3 = (d1E * d1E - SK_LEFT_ROOT_ARM_LENGTH_MM * SK_LEFT_ROOT_ARM_LENGTH_MM -
    //              SK_LEFT_FAR_ARM_LENGTH_MM * SK_LEFT_FAR_ARM_LENGTH_MM) /
    //             (2.0 * SK_LEFT_ROOT_ARM_LENGTH_MM * SK_LEFT_FAR_ARM_LENGTH_MM);
    //    cosQ4 = (d2E * d2E - SK_RIGHT_ROOT_ARM_LENGTH_MM * SK_RIGHT_ROOT_ARM_LENGTH_MM -
    //              SK_RIGHT_FAR_ARM_LENGTH_MM * SK_RIGHT_FAR_ARM_LENGTH_MM) /
    //             (2.0 * SK_RIGHT_ROOT_ARM_LENGTH_MM * SK_RIGHT_FAR_ARM_LENGTH_MM);
    //

    cosU1 = (d1E * d1E + a1 * a1 - a3 * a3) / (2.0 * a1 * d1E);
    cosU2 = (d2E * d2E + a2 * a2 - a4 * a4) / (2.0 * a2 * d2E);

    if ((cosU1 < -1.0) || (cosU1 > 1.0) || (cosU2 < -1.0) || (cosU2 > 1.0))
    {
        return kStatus_Fail;
    }

    if (point.y >= 0.0)
    {
        q1 = atan2(robotPoint.y, robotPoint.x) + acos(cosU1);
        q2 = atan2(robotPoint.y, robotPoint.x - c) - acos(cosU2);
    }
    else
    {
        q1 = atan2(robotPoint.y, robotPoint.x) - acos(cosU1);
        q2 = atan2(robotPoint.y, robotPoint.x - c) + acos(cosU2);
    }

    if ((!isfinite(q1)) || (!isfinite(q2)))
    {
        return kStatus_Fail;
    }
    double q1normalized = q1;
    double q2normalized = q2;

    for (int i = 0; i < 2; i++)
    {
        if (q1normalized < 0.0)
        {
            q1normalized += 2.0 * M_PI;
        }
        else if (q1normalized > 2 * M_PI)
        {
            q1normalized -= 2.0 * M_PI;
        }

        if (q2normalized < 0.0)
        {
            q2normalized += 2.0 * M_PI;
        }
        else if (q2normalized > 2 * M_PI)
        {
            q2normalized -= 2.0 * M_PI;
        }
    }

    if (zetaE != NULL)
    {
        SK_Point_t point;
        SK_forward_kinematics(q1normalized, q2normalized,
                              robotPoint.y >= 0.0 ? SK_SIDE_MAIN : SK_SIDE_MIRRORED, &robotPoint,
                              zetaE);
    }

    if (theta1 != NULL)
    {
        *theta1 = q1normalized;
    }
    if (theta2 != NULL)
    {
        *theta2 = q2normalized;
    }
    return kStatus_Success;
}

status_t SK_forward_kinematics(double theta1, double theta2, SK_Side_t side, SK_Point_t* point,
                               double* zetaE)
{
    const double c  = SK_BASE_SPAN_MM;
    const double a1 = SK_LEFT_ROOT_ARM_LENGTH_MM;
    const double a2 = SK_RIGHT_ROOT_ARM_LENGTH_MM;
    const double a3 = SK_LEFT_FAR_ARM_LENGTH_MM;
    const double a4 = SK_RIGHT_FAR_ARM_LENGTH_MM;

    if (point == NULL)
    {
        return kStatus_Fail;
    }

    if ((a1 <= 0.0) || (a2 <= 0.0) || (a3 <= 0.0) || (a4 <= 0.0))
    {
        return kStatus_Fail;
    }

    const SK_Point_t j1 = {
        .x = 0.0,
        .y = 0.0,
    };

    const SK_Point_t j2 = {
        .x = c,
        .y = 0.0,
    };

    const SK_Point_t j3 = {
        .x = a1 * cos(theta1),
        .y = a1 * sin(theta1),
    };

    const SK_Point_t j4 = {
        .x = c + (a2 * cos(theta2)),
        .y = a2 * sin(theta2),
    };

    const double dx        = j4.x - j3.x;
    const double dy        = j4.y - j3.y;
    const double d         = hypot(dx, dy);
    const double q3BetaCos = ((a3 * a3) + (d * d) - (a4 * a4)) / (2.0 * a3 * d);

    (void)j1;
    (void)j2;

    if ((d <= 0.0) || (d > (a3 + a4)))
    {
        return kStatus_Fail;
    }

    if ((q3BetaCos < -1.0) || (q3BetaCos > 1.0))
    {
        return kStatus_Fail;
    }

    const double alpha = atan2(dy, dx);
    const double beta  = acos(q3BetaCos);

    if ((!isfinite(alpha)) || (!isfinite(beta)))
    {
        return kStatus_Fail;
    }

    SK_Point_t robotPoint;
    if (side == SK_SIDE_MAIN)
    {
        robotPoint.x = j3.x + (a3 * cos(alpha + beta));
        robotPoint.y = j3.y + (a3 * sin(alpha + beta));

        if (zetaE != NULL)
        {
            *zetaE = M_PI - beta + alpha;
        }
    }
    else
    {
        robotPoint.x = j3.x - (a3 * cos(alpha + beta));
        robotPoint.y = j3.y - (a3 * sin(alpha + beta));

        if (zetaE != NULL)
        {
            *zetaE = M_PI + alpha + beta;
        }
    }

    SK_Point_t topLevelPoint;
    if (SKi_inverse_transform(&robotPoint, &topLevelPoint) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    *point = topLevelPoint;
    return kStatus_Success;
}

status_t SK_move_to_xy_async(SK_Point_t targetPoint, double deltaZetaE, bool rotateZeta,
                             TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    SK_CmdQueueItem_t queueItem = {
        .type                      = SK_CMD_MOVE_TO_XY,
        .deadline                  = deadline,
        .cmdHandle                 = NULL,
        .data.moveToXY.targetPoint = targetPoint,
        .data.moveToXY.deltaZetaE  = deltaZetaE,
        .data.moveToXY.rotateZeta  = rotateZeta,
    };

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t SK_home(const SK_Point_t zeroOffset, TickType_t deadline, CHD_CmdHandle_t* cmdHandle)
{
    SK_CmdQueueItem_t queueItem = {
        .type                 = SK_CMD_HOME,
        .deadline             = deadline,
        .cmdHandle            = NULL,
        .data.home.zeroOffset = zeroOffset,
    };

    return SKi_send_cmd_async(&queueItem, deadline, cmdHandle);
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
static status_t SKi_apply_transform(const SK_Point_t* topLevelPoint, SK_Point_t* robotPoint)
{
    if ((topLevelPoint == NULL) || (robotPoint == NULL))
    {
        return kStatus_Fail;
    }

    robotPoint->x = (topLevelPoint->x * skState.transform.scaleX) + skState.transform.offsetX;
    robotPoint->y = (topLevelPoint->y * skState.transform.scaleY) + skState.transform.offsetY;
    return kStatus_Success;
}

static status_t SKi_inverse_transform(const SK_Point_t* robotPoint, SK_Point_t* topLevelPoint)
{
    if ((robotPoint == NULL) || (topLevelPoint == NULL))
    {
        return kStatus_Fail;
    }

    if ((skState.transform.scaleX == 0.0) || (skState.transform.scaleY == 0.0))
    {
        return kStatus_Fail;
    }

    topLevelPoint->x = (robotPoint->x - skState.transform.offsetX) / skState.transform.scaleX;
    topLevelPoint->y = (robotPoint->y - skState.transform.offsetY) / skState.transform.scaleY;
    return kStatus_Success;
}

static bool SKi_point_in_rect(const SK_WorkspaceRect_t* rect, const SK_Point_t* point)
{
    if ((rect == NULL) || (point == NULL))
    {
        return false;
    }

    return (point->x >= rect->minX) && (point->x <= rect->maxX) && (point->y >= rect->minY) &&
           (point->y <= rect->maxY);
}

static status_t SKi_workspace_allowed(const SK_Point_t* point, bool* allowed)
{
    uint8_t i;

    if ((point == NULL) || (allowed == NULL))
    {
        return kStatus_Fail;
    }

    if ((!skState.workspaceLimits.enabled) || (skState.workspaceLimits.rectangleCount == 0U))
    {
        *allowed = true;
        return kStatus_Success;
    }

    *allowed = false;
    for (i = 0U; i < skState.workspaceLimits.rectangleCount; ++i)
    {
        if (SKi_point_in_rect(&skState.workspaceLimits.rectangles[i], point))
        {
            *allowed = true;
            break;
        }
    }

    return kStatus_Success;
}
