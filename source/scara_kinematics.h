/************************************************************
 * @file    scara_kinematics.h
 * @brief   Filedescription
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

typedef enum _SK_Configuration
{
    SK_BOTH_ELBOW_OUT,
    SK_L_ELBOW_OUT_R_ELBOW_IN,
    SK_L_ELBOW_IN_R_ELBOW_OUT,
    SK_BOTH_ELBOW_IN
} SK_Configuration_t;

/****************************
 *     Public Variables     *
 ****************************/

/**************************************
 *     Public Function Prototypes    *
 **************************************/

#endif // SCARA_KINEMATICS_H_