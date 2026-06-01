/************************************************************
 * @file    scara_kinematics_internal.h
 * @brief   Internal command types and shared state for the kinematics module.
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

#ifndef MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_INTERNAL_H_
#define MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************
 *     Includes    *
 ********************/
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "cmd_handle.h"
#include <motion/scara_kinematics/scara_kinematics.h>

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

#define SK_CMD_QUEUE_SIZE (16U)
#define SK_MAX_CMD_HANDLE_COUNT (16U)

    /***************************
     *     Public Typedefs     *
     ***************************/

    typedef enum _SK_CmdType
    {
        SK_CMD_SET_TRANSFORM,
        SK_CMD_SET_WORKSPACE_LIMITS,
        SK_CMD_ENABLE_WORKSPACE_LIMITS,
        SK_CMD_SET_ORIGIN_OFFSET,
        SK_CMD_CHANGE_SIDE,
        SK_CMD_MOVE_TO_XY,
        SK_CMD_HOME,
    } SK_CmdType_t;

    typedef struct _SK_SetTransformData
    {
        SK_Transform_t transform;
    } SK_SetTransformData_t;

    typedef struct _SK_SetWorkspaceLimitsData
    {
        SK_WorkspaceLimits_t limits;
    } SK_SetWorkspaceLimitsData_t;

    typedef struct _SK_EnableWorkspaceLimitsData
    {
        bool enabled;
    } SK_EnableWorkspaceLimitsData_t;

    typedef struct _SK_SetOriginOffsetData
    {
        SK_Point_t offset;
    } SK_SetOriginOffsetData_t;

    typedef struct _SK_MoveToXYData
    {
        SK_Point_t targetPoint;
        double     deltaZetaE; /*< Optional change in end-effector angle, where positive is
                                  counterclockwise. */
        bool rotateZeta;       /*< Whether to rotate the end-effector */
    } SK_MoveToXYData_t;

    typedef struct _SK_HomeData
    {
        SK_Point_t zeroOffset;
    } SK_HomeData_t;

    typedef struct _SK_CmdQueueItem
    {
        SK_CmdType_t    type;
        TickType_t      deadline;
        CHD_CmdHandle_t cmdHandle;
        union
        {
            SK_SetTransformData_t          setTransform;
            SK_SetWorkspaceLimitsData_t    setWorkspaceLimits;
            SK_EnableWorkspaceLimitsData_t enableWorkspaceLimits;
            SK_SetOriginOffsetData_t       setOriginOffset;
            SK_MoveToXYData_t              moveToXY;
            SK_HomeData_t                  home;
        } data;
    } SK_CmdQueueItem_t;

    /****************************
     *     Public Variables     *
     ****************************/

    extern QueueHandle_t       skCmdQueue;
    extern CHD_CmdHandleImpl_t skCmdHandles[SK_MAX_CMD_HANDLE_COUNT];
    extern SK_State_t          skState;

    extern MTR_MotorHandle_t skLeftMotorHandle;
    extern MTR_MotorHandle_t skRightMotorHandle;
    extern MTR_MotorHandle_t skZetaMotorHandle;

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

#ifdef __cplusplus
}
#endif

#endif // MOTION_SCARA_KINEMATICS_SCARA_KINEMATICS_INTERNAL_H_
