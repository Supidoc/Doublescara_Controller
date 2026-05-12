/************************************************************
 * @file    grabber.h
 * @brief   Motor grabber latch helpers.
 * @author  dg
 * @date    27 Apr 2026
 ************************************************************/
#ifndef GRABBER_H_
#define GRABBER_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes    *
     ********************/

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

    void GRB_grab(void);

    void GRB_release(void);

#ifdef __cplusplus
}
#endif

#endif /* GRABBER_H_ */
