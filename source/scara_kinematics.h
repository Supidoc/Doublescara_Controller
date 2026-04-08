/************************************************************
 * @file    scara_kinematics.h
 * @brief   SCARA robot arm inverse and forward kinematics
 * @details Provides kinematics calculations for a SCARA (Selective Compliance
 *          Articulated Robot Arm) manipulator with two articulated arms.
 *          Supports multiple elbow configurations for singularity resolution.
 * @author  dg
 * @date    17 Mar 2026
 ************************************************************/

#ifndef SCARA_KINEMATICS_H_
#define SCARA_KINEMATICS_H_

/********************
 *     Includes    *
 ********************/

/***********************************
 *     Public Macros / Defines     *
 ***********************************/

/***************************
 *     Public Typedefs     *
 ***************************/

/**
 * @brief SCARA elbow configuration options
 * @details Specifies which elbow configuration to use for inverse kinematics.
 *          Different configurations are used to resolve the elbow-up or elbow-down
 *          solution ambiguity in SCARA arm kinematics.
 */
typedef enum _SK_Configuration
{
    SK_BOTH_ELBOW_OUT,         /**< Both elbows pointing outward */
    SK_L_ELBOW_OUT_R_ELBOW_IN, /**< Left elbow out, right elbow in */
    SK_L_ELBOW_IN_R_ELBOW_OUT, /**< Left elbow in, right elbow out */
    SK_BOTH_ELBOW_IN           /**< Both elbows pointing inward */
} SK_Configuration_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

#endif // SCARA_KINEMATICS_H_