/************************************************************
 * @file    scara_kinematics.c
 * @brief   Filedescription
 * @author  dg
 * @date    17 Mar 2026
 ************************************************************/

/********************
 *     Includes    *
 ********************/
#include "scara_kinematics.h"
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

status_t scara_inverse_kinematics(double x, double y, SK_Configuration_t configuration,
                                  double* theta1, double* theta2)
{

    d1E    = sqrt(pow(x, 2) + pow(y, 2));
    d2E    = sqrt(pow(x - c, 2) + pow(y, 2));
    alpha1 = acos((pow(d1E, 2) + pow(a1, 2) - pow(a3, 2)) / (2 * a1 * d1E));
    alpha2 = acos((pow(d2E, 2) + pow(a2, 2) - pow(a4, 2)) / (2 * a2 * d2E));
    beta1  = atan2(y, x);
    beta2  = atan2(y, x - c);

    if (isnan(alpha1) || isnan(alpha2) || isinf(alpha1) || isinf(alpha2))
    {
        return kStatus_Fail; // Position is unreachable
    }

    if (isnan(beta1) || isnan(beta2) || isinf(beta1) || isinf(beta2))
    {
        return kStatus_Fail; // Position is unreachable
    }

    switch (configuration)
    {
        case SK_BOTH_ELBOW_OUT:
            theta1Temp = beta1 - alpha1;
            theta2Temp = beta2 - alpha2;
            break;
        case SK_L_ELBOW_OUT_R_ELBOW_IN:
            theta1Temp = beta1 - alpha1;
            theta2Temp = beta2 + alpha2;
            break;
        case SK_L_ELBOW_IN_R_ELBOW_OUT:
            theta1Temp = beta1 + alpha1;
            theta2Temp = beta2 - alpha2;
            break;
        case SK_BOTH_ELBOW_IN:
            theta1Temp = beta1 + alpha1;
            theta2Temp = beta2 + alpha2;
            break;
    }

    *theta1 = theta1Temp;
    *theta2 = theta2Temp;

    return kStatus_Success;
}

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

/********************************************
 *     Private Function Implementations     *
 ********************************************/
