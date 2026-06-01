/************************************************************
 * @file    grabber.h
 * @brief   Motor grabber latch helpers.
 * @author  dg
 * @date    27 Apr 2026
 ************************************************************/
#ifndef MOTION_GRABBER_GRABBER_H_
#define MOTION_GRABBER_GRABBER_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes    *
     ********************/

#include "fsl_common.h"

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

    /***************************
     *     Public Typedefs     *
     ***************************/

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    status_t GRB_platform_up(void);

    status_t GRB_platform_down(void);

    status_t GRB_magnet_up(void);

    status_t GRB_magnet_down(void);

    status_t GRB_grab(void);

    status_t GRB_release(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_GRABBER_GRABBER_H_ */
