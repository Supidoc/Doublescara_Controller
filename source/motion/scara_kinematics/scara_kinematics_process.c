/************************************************************
 * @file    scara_kinematics_process.c
 * @brief   Kinematics command queue processing and async dispatch
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include <motion/scara_kinematics/scara_kinematics.h>
#include <motion/scara_kinematics/scara_kinematics_internal.h>
#include <motion/scara_kinematics/scara_kinematics_process.h>
#include "cmd_dispatch.h"
#include "math.h"
#include "sync_wait.h"
#include "motor_configs.h"

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/
static status_t SKi_handle_set_transform(const SK_SetTransformData_t* data);
static status_t SKi_handle_set_workspace_limits(const SK_SetWorkspaceLimitsData_t* data);
static status_t SKi_handle_enable_workspace_limits(const SK_EnableWorkspaceLimitsData_t* data);
static status_t SKi_handle_set_origin_offset(const SK_SetOriginOffsetData_t* data);
static status_t SKi_handle_change_side(TickType_t deadline);
static status_t SKI_handle_MoveToXY(const SK_MoveToXYData_t* data, TickType_t deadline);
static status_t SKi_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline);
static status_t SKI_side_change_required(SK_Point_t startPoint, SK_Point_t targetPoint,
                                         bool* required);

static double SKi_rad_to_deg(double radians);
static double SKi_deg_to_rad(double degrees);

/****************************
 *     Public Variables     *
 ****************************/

QueueHandle_t       skCmdQueue                            = NULL;
CHD_CmdHandleImpl_t skCmdHandles[SK_MAX_CMD_HANDLE_COUNT] = {0};

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t SKi_init_queue(void)
{
    if (skCmdQueue != NULL)
    {
        return kStatus_Success;
    }

    CHD_init_cmd_handles(skCmdHandles, SK_MAX_CMD_HANDLE_COUNT);

    skCmdQueue = xQueueCreate(SK_CMD_QUEUE_SIZE, sizeof(SK_CmdQueueItem_t));
    if (skCmdQueue == NULL)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t SKi_send_cmd_async(SK_CmdQueueItem_t* queueItem, TickType_t deadline,
                            CHD_CmdHandle_t* cmdHandle)
{
    if ((queueItem == NULL) || (skCmdQueue == NULL))
    {
        return kStatus_Fail;
    }

    CHD_CmdHandle_t internalCmdHandle = NULL;

    status_t allocStatus =
        CHD_get_cmd_handle(&internalCmdHandle, skCmdHandles, SK_MAX_CMD_HANDLE_COUNT);
    if (allocStatus != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (cmdHandle != NULL)
    {
        CHD_add_cmd_handle_ref(internalCmdHandle);
    }

    queueItem->cmdHandle = internalCmdHandle;

    status_t sendStatus = CDP_send_cmd(skCmdQueue, queueItem, deadline, internalCmdHandle);
    if (sendStatus != kStatus_Success)
    {
        if (cmdHandle != NULL)
        {
            CHD_remove_cmd_handle_ref(internalCmdHandle);
            *cmdHandle = NULL;
        }
        CHD_remove_cmd_handle_ref(internalCmdHandle);
        return kStatus_Fail;
    }

    if (cmdHandle != NULL)
    {
        *cmdHandle = internalCmdHandle;
    }

    return kStatus_Success;
}

void SKi_process_cmd(SK_CmdQueueItem_t queueItem)
{
    status_t result = kStatus_Fail;

    switch (queueItem.type)
    {
        case SK_CMD_SET_TRANSFORM:
            result = SKi_handle_set_transform(&queueItem.data.setTransform);
            break;

        case SK_CMD_SET_WORKSPACE_LIMITS:
            result = SKi_handle_set_workspace_limits(&queueItem.data.setWorkspaceLimits);
            break;

        case SK_CMD_SET_ORIGIN_OFFSET:
            result = SKi_handle_set_origin_offset(&queueItem.data.setOriginOffset);
            break;

        case SK_CMD_CHANGE_SIDE:
            result = SKi_handle_change_side(queueItem.deadline);
            break;
        case SK_CMD_MOVE_TO_XY:
            result = SKI_handle_MoveToXY(&queueItem.data.moveToXY, queueItem.deadline);
            break;
        default:
            result = kStatus_Fail;
            break;
    }

    if (result == kStatus_Success)
    {
        CDP_notify_task_success(queueItem.cmdHandle);
    }
    else
    {
        CDP_notify_task_failure(queueItem.cmdHandle);
    }

    CHD_remove_cmd_handle_ref(queueItem.cmdHandle);
}

void SKi_task(void* pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        SK_CmdQueueItem_t queueItem = {0};
        BaseType_t        status    = xQueueReceive(skCmdQueue, &queueItem, portMAX_DELAY);

        if (status == pdPASS)
        {
            SKi_process_cmd(queueItem);
        }
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static status_t SKi_handle_set_transform(const SK_SetTransformData_t* data)
{
    if (data == NULL)
    {
        return kStatus_Fail;
    }

    skState.transform = data->transform;

    return kStatus_Success;
}

static status_t SKi_handle_set_workspace_limits(const SK_SetWorkspaceLimitsData_t* data)
{
    if (data == NULL)
    {
        return kStatus_Fail;
    }

    skState.workspaceLimits = data->limits;

    return kStatus_Success;
}

static status_t SKi_handle_enable_workspace_limits(const SK_EnableWorkspaceLimitsData_t* data)
{
    if (data == NULL)
    {
        return kStatus_Fail;
    }

    skState.workspaceLimits.enabled = data->enabled;

    return kStatus_Success;
}

static status_t SKi_handle_set_origin_offset(const SK_SetOriginOffsetData_t* data)
{
    if (data == NULL)
    {
        return kStatus_Fail;
    }

    SK_State_t currentState        = skState;
    currentState.transform.offsetX = data->offset.x;
    currentState.transform.offsetY = data->offset.y;
    skState                        = currentState;

    return kStatus_Success;
}

static status_t SKi_handle_change_side(TickType_t deadline)
{
    SK_State_t currentState = skState;

    double theta1Deg;
    double theta2Deg;
    if (currentState.side == SK_SIDE_MAIN)
    {
        theta1Deg = 135.0;
    }
    else
    {
        theta1Deg = 225.0;
    }

    theta2Deg = 16;

    double currentTheta1Deg = 0;
    double currentTheta2Deg = 0;
    MTR_get_current_angle_async(skRightMotorHandle, &currentTheta1Deg, 0, NULL);
    MTR_get_current_angle_async(skLeftMotorHandle, &currentTheta2Deg, 0, NULL);

    double relativeTheta1Deg = theta1Deg - currentTheta1Deg;
    double relativeTheta2Deg = theta2Deg - currentTheta2Deg;

    MTR_MotorHandle_t handles[2];
    handles[0] = skRightMotorHandle;
    handles[1] = skLeftMotorHandle;
    double angles[2];
    angles[0] = relativeTheta1Deg;
    angles[1] = relativeTheta2Deg;

    CHD_CmdHandle_t moveCmdHandle = NULL;

    if ((currentTheta1Deg > theta1Deg && currentState.side == SK_SIDE_MAIN) ||
        (currentTheta1Deg < theta1Deg && currentState.side == SK_SIDE_MIRRORED))
    {
        if (MTR_synchronized_move_async(handles, angles, 2U, portMAX_DELAY, &moveCmdHandle) !=
            kStatus_Success)
        {
            return kStatus_Fail;
        }
    }
    else
    {
        MTR_move_absolute_angle_async(skLeftMotorHandle, theta2Deg, deadline, &moveCmdHandle);
    }
    if (SKi_wait_and_release(moveCmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    CHD_CmdHandle_t moveCmdHandle2 = NULL;

    if (currentState.side == SK_SIDE_MAIN)
    {
        theta1Deg = 225.0;
        theta2Deg = -45.0;
    }
    else
    {
        theta1Deg = 135.0;
        theta2Deg = 45.0;
    }

    if (MTR_move_absolute_angle_async(skRightMotorHandle, theta1Deg, deadline, &moveCmdHandle) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (SKi_wait_and_release(moveCmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    if (MTR_move_absolute_angle_async(skLeftMotorHandle, theta2Deg, deadline, &moveCmdHandle) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (SKi_wait_and_release(moveCmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    currentState.side = (currentState.side == SK_SIDE_MAIN) ? SK_SIDE_MIRRORED : SK_SIDE_MAIN;
    skState           = currentState;

    return kStatus_Success;
}

static status_t SKI_handle_MoveToXY(const SK_MoveToXYData_t* data, TickType_t deadline)
{
    if (data == NULL)
    {
        return kStatus_Fail;
    }

    bool       sideChangeRequired = false;
    SK_Point_t startPoint         = {
        .x = skState.pose.x,
        .y = skState.pose.y,
    };

    double zetaEStartRad;
    double zetaEEndRad;

    SK_inverse_kinematics(startPoint, NULL, NULL, &zetaEStartRad);

    double theta1Rad;
    double theta2Rad;

    if (SK_inverse_kinematics(data->targetPoint, &theta1Rad, &theta2Rad, &zetaEEndRad) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }

    double          deltaZetaEDEG     = 0.0;
    CHD_CmdHandle_t zetaMoveCmdHandle = NULL;
    if (data->rotateZeta)
    {
        deltaZetaEDEG = data->deltaZetaE + SKi_rad_to_deg(zetaEStartRad - zetaEEndRad);
        //        MTR_move_angle_async(skZetaMotorHandle, deltaZetaEDEG, deadline,
        //                             &zetaMoveCmdHandle);
    }

    if (SKI_side_change_required(startPoint, data->targetPoint, &sideChangeRequired) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (sideChangeRequired)
    {
        SKi_handle_change_side(deadline);
    }

    double absoluteTheta1Deg = SKi_rad_to_deg(theta1Rad);
    double absoluteTheta2Deg = SKi_rad_to_deg(theta2Rad);

    // right arm should move from 180 to -180 degrees
    if (absoluteTheta2Deg > 180.0)
    {
        absoluteTheta2Deg -= 360.0;
    }

    double currentTheta1Deg = 0;
    double currentTheta2Deg = 0;
    MTR_get_current_angle_async(skRightMotorHandle, &currentTheta1Deg, 0, NULL);
    MTR_get_current_angle_async(skLeftMotorHandle, &currentTheta2Deg, 0, NULL);

    double relativeTheta1Deg = absoluteTheta1Deg - currentTheta1Deg;
    double relativeTheta2Deg = absoluteTheta2Deg - currentTheta2Deg;

    CHD_CmdHandle_t   moveCmdHandle = NULL;
    MTR_MotorHandle_t handles[2];
    handles[0] = skRightMotorHandle;
    handles[1] = skLeftMotorHandle;
    // handles[2] = skZetaMotorHandle;
    double angles[3];
    angles[0] = relativeTheta1Deg;
    angles[1] = relativeTheta2Deg;
    // angles[2] = deltaZetaEDEG;
    if (MTR_synchronized_move_async(handles, angles, 2, portMAX_DELAY, &moveCmdHandle) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (SKi_wait_and_release(moveCmdHandle, deadline) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (data->rotateZeta && fabs(data->deltaZetaE) > 0.05)
    {
        if (MTR_move_angle_async(skZetaMotorHandle, deltaZetaEDEG, deadline, &moveCmdHandle) !=
            kStatus_Success)
        {
            return kStatus_Fail;
        }
        if (SKi_wait_and_release(moveCmdHandle, deadline) != kStatus_Success)
        {
            return kStatus_Fail;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    skState.pose.x = data->targetPoint.x;
    skState.pose.y = data->targetPoint.y;

    return kStatus_Success;
}

static status_t SKI_side_change_required(SK_Point_t startPoint, SK_Point_t targetPoint,
                                         bool* required)
{
    if (startPoint.y < 0.0 && targetPoint.y >= 0.0)
    {
        *required = true;
    }
    else if (startPoint.y >= 0.0 && targetPoint.y < 0.0)
    {
        *required = true;
    }
    else
    {
        *required = false;
    }

    return kStatus_Success;
}

static status_t SKi_wait_and_release(CHD_CmdHandle_t cmdHandle, TickType_t deadline)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    status_t status = SYW_cmd_wait_result(cmdHandle, deadline, NULL);
    CHD_remove_cmd_handle_ref(cmdHandle);
    return status;
}

static inline double SKi_rad_to_deg(double radians)
{
    return radians * (180.0 / M_PI);
}

static inline double SKi_deg_to_rad(double degrees)
{
    return degrees * (M_PI / 180.0);
}
