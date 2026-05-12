/************************************************************
 * @file    scara_kinematics.h
 * @brief   Five-bar SCARA kinematics and workspace state
 * @details Draft API for a stateful five-bar robot kinematics module.
 *          Geometry is configured by defines, while scaling, offset,
 *          workspace limits, and pose state are tracked at runtime.
 * @author  dg
 * @date    17 Apr 2026
 ************************************************************/

#ifndef SCARA_KINEMATICS_H_
#define SCARA_KINEMATICS_H_

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
#include "fsl_common.h"
#include "cmd_handle.h"
#include "motor_core.h"

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/** @brief Length of the left arm's root segment in millimeters (configurable). */
#ifndef SK_LEFT_ROOT_ARM_LENGTH_MM
#define SK_LEFT_ROOT_ARM_LENGTH_MM (150.0)
#endif

/** @brief Length of the left arm's far segment in millimeters (configurable). */
#ifndef SK_LEFT_FAR_ARM_LENGTH_MM
#define SK_LEFT_FAR_ARM_LENGTH_MM (175.0)
#endif

/** @brief Length of the right arm's root segment in millimeters (configurable). */
#ifndef SK_RIGHT_ROOT_ARM_LENGTH_MM
#define SK_RIGHT_ROOT_ARM_LENGTH_MM (150.0)
#endif

/** @brief Length of the right arm's far segment in millimeters (configurable). */
#ifndef SK_RIGHT_FAR_ARM_LENGTH_MM
#define SK_RIGHT_FAR_ARM_LENGTH_MM (175.0)
#endif

/** @brief Distance between the left and right arm base points in millimeters (configurable). */
#ifndef SK_BASE_SPAN_MM
#define SK_BASE_SPAN_MM (60.0)
#endif

/** @brief Maximum number of workspace rectangles supported. */
#define SK_MAX_WORKSPACE_RECTS (8U)

    /***************************
     *     Public Typedefs     *
     ***************************/

    /**
     * @brief Represents a 2D Cartesian point in top-level coordinates.
     *
     * @details Used to represent end-effector positions and targets in the workspace.
     */
    typedef struct _SK_Point
    {
        double x; /*< Horizontal position in millimeters */
        double y; /*< Vertical position in millimeters */
    } SK_Point_t;

    /**
     * @brief Represents the current end-effector pose (position).
     *
     * @details Stores the current Cartesian position of the end-effector
     *          in top-level coordinates after applying all transforms.
     */
    typedef struct _SK_Pose
    {
        double x; /*< End-effector x position in top-level coordinates (mm) */
        double y; /*< End-effector y position in top-level coordinates (mm) */
    } SK_Pose_t;

    /**
     * @brief Coordinate transformation parameters for workspace scaling and offset.
     *
     * @details Defines linear scaling and translation applied to convert from
     *          mechanical joint angles to top-level Cartesian coordinates.
     */
    typedef struct _SK_Transform
    {
        double scaleX;  /*< Horizontal scaling factor (dimensionless) */
        double scaleY;  /*< Vertical scaling factor (dimensionless) */
        double offsetX; /*< Horizontal offset applied after scaling (mm) */
        double offsetY; /*< Vertical offset applied after scaling (mm) */
    } SK_Transform_t;

    /**
     * @brief Defines a rectangular region within the workspace.
     *
     * @details A rectangular constraint used to limit valid motion to safe areas.
     *          Coordinates are in top-level space before scaling is applied.
     */
    typedef struct _SK_WorkspaceRect
    {
        double minX; /*< Minimum x coordinate of the rectangle (mm) */
        double minY; /*< Minimum y coordinate of the rectangle (mm) */
        double maxX; /*< Maximum x coordinate of the rectangle (mm) */
        double maxY; /*< Maximum y coordinate of the rectangle (mm) */
    } SK_WorkspaceRect_t;

    /**
     * @brief Defines a collection of rectangular workspace limits.
     *
     * @details Stores multiple rectangular regions that define safe motion areas.
     *          When enabled, the kinematics layer checks that all targets lie within
     *          at least one of the defined rectangles.
     */
    typedef struct _SK_WorkspaceLimits
    {
        SK_WorkspaceRect_t rectangles[SK_MAX_WORKSPACE_RECTS]; /*< Array of workspace rectangles */
        uint8_t            rectangleCount;                     /*< Number of active rectangles */
        bool               enabled; /*< Whether workspace limit checking is active */
    } SK_WorkspaceLimits_t;

    /**
     * @brief Enumerates the two possible robot configurations (main or mirrored).
     *
     * @details Allows switching between a standard and horizontally-flipped configuration
     *          to reach different areas of the workspace.
     */
    typedef enum _SK_Side
    {
        SK_SIDE_MAIN     = 0, /*< Standard robot configuration */
        SK_SIDE_MIRRORED = 1, /*< Horizontally-flipped configuration */
    } SK_Side_t;

    /**
     * @brief Stores the complete kinematics state including pose, transforms, and limits.
     *
     * @details Maintains the runtime configuration and state of the kinematics module,
     *          including the current end-effector pose, coordinate transformation parameters,
     *          workspace constraints, and calibration status.
     */
    typedef struct _SK_State
    {
        SK_Pose_t            pose;      /*< Current end-effector pose in top-level coordinates */
        SK_Transform_t       transform; /*< Current coordinate transformation parameters */
        SK_WorkspaceLimits_t workspaceLimits; /*< Workspace limitation configuration */
        SK_Side_t            side;            /*< Current robot configuration (main or mirrored) */
        bool                 calibrated;      /*< Whether the robot has been homed and calibrated */
    } SK_State_t;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief Initializes the kinematics module and command queue.
     *
     * @return kStatus_Success if initialization succeeds.
     */
    status_t SK_init(void);

    /**
     * @brief Stores the motor handles used by the kinematics layer.
     *
     * @param[in] leftMotorHandle Handle for the left motor.
     * @param[in] rightMotorHandle Handle for the right motor.
     * @param[in] zetaMotorHandle Handle for the zeta (end-effector rotation) motor.
     * @return kStatus_Success if the operation succeeds.
     */
    status_t SK_init_motor_handles(MTR_MotorHandle_t leftMotorHandle,
                                   MTR_MotorHandle_t rightMotorHandle,
                                   MTR_MotorHandle_t zetaMotorHandle);

    /**
     * @brief Task function for processing kinematics commands.
     *
     * Should be started as a FreeRTOS task after SK_init().
     *
     * @param[in] pvParameters Pointer to task parameters (not used).
     */
    void SK_task(void* pvParameters);

    /**
     * @brief Queues an update of the runtime coordinate transform.
     *
     * @param[in] transform New scale and offset.
     * @param[in] deadline Deadline for the command.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if the command was accepted.
     */
    status_t SK_set_transform_async(SK_Transform_t transform, TickType_t deadline,
                                    CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Queues an update of the workspace limits.
     *
     * Rectangle limits are interpreted in top-level coordinates before scaling.
     *
     * @param[in] limits New rectangle set.
     * @param[in] deadline Deadline for the command.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if the command was accepted.
     */
    status_t SK_set_workspace_limits_async(SK_WorkspaceLimits_t limits, TickType_t deadline,
                                           CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Enables or disables workspace limit checking.
     *
     * @param[in] enabled New enable state.
     * @param[in] deadline Deadline for the command.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if the command was accepted.
     */
    status_t SK_enable_workspace_limits_async(bool enabled, TickType_t deadline,
                                              CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Queues an update of the zero-point offset.
     *
     * @param[in] offset New zero offset in top-level coordinates.
     * @param[in] deadline Deadline for the command.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if the command was accepted.
     */
    status_t SK_set_origin_offset_async(SK_Point_t offset, TickType_t deadline,
                                        CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Placeholder for switching to the mirrored side.
     *
     * The motion layer will later provide the path execution. The kinematics layer
     * will only update side state and mirrored target data.
     *
     * @param[in] deadline Deadline for the command.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if the command was accepted.
     */
    status_t SK_change_side_async(TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Checks whether a point is allowed by the configured workspace limits.
     *
     * @param[in] point Top-level point to validate.
     * @param[out] allowed True if the point lies inside any enabled rectangle.
     * @return kStatus_Success if the check completed.
     */
    status_t SK_is_point_allowed(SK_Point_t point, bool* allowed);

    /**
     * @brief Solves inverse kinematics for the current geometry.
     *
     * @param[in] point Top-level point for which to solve inverse kinematics.
     * @param[out] theta1 Left motor angle in radians.
     * @param[out] theta2 Right motor angle in radians.
     * @param[out] zetaE Optional pointer to end-effector angle in radians, where 0 is parallel to
     * the base and positive is counterclockwise.
     * @return kStatus_Success if the target is reachable.
     */
    status_t SK_inverse_kinematics(SK_Point_t point, double* theta1, double* theta2, double* zetaE);

    /**
     * @brief Solves forward kinematics for the current geometry.
     *
     * @param[in] theta1 Left motor angle in radians.
     * @param[in] theta2 Right motor angle in radians.
     * @param[in] side Current robot side (main or mirrored).
     * @param[out] point Pointer to the resulting top-level point.
     * @param[out] zetaE Optional pointer to end-effector angle in radians, where 0 is parallel to
     * the base and positive is counterclockwise.
     * @return kStatus_Success if a valid Cartesian point was found.
     */
    status_t SK_forward_kinematics(double theta1, double theta2, SK_Side_t side, SK_Point_t* point,
                                   double* zetaE);

    /**
     * @brief Executes a Cartesian move through the motor module and waits for completion.
     *
     * This helper updates kinematics commanded state, executes a synchronized motor move,
     * and refreshes the actual pose from motor feedback.
     *
     * @param[in] targetPoint Top-level x/y target.
     * @param[in] deltaZetaE Optional change in end-effector angle, where positive is
     * counterclockwise. Only applied if rotateZeta is true.
     * @param[in] rotateZeta Whether to rotate the end-effector by deltaZetaE.
     * @param[in] deadline Deadline for all queued operations.
     * @return kStatus_Success if the full sequence succeeds.
     */
    status_t SK_move_to_xy_async(SK_Point_t targetPoint, double deltaZetaE, bool rotateZeta,
                                 TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

    /**
     * @brief Runs homing through the homing module.
     *
     * @param[in] zeroOffset Zero-point offset to apply after homing, in top-level coordinates.
     * @param[in] deadline Deadline for the homing operation.
     * @param[out] cmdHandle Optional command handle used to wait for completion.
     * @return kStatus_Success if homing succeeds.
     */
    status_t SK_home(const SK_Point_t zeroOffset, TickType_t deadline, CHD_CmdHandle_t* cmdHandle);

#ifdef __cplusplus
}
#endif

#endif // SCARA_KINEMATICS_H_
