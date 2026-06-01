/************************************************************
 * @file    init.h
 * @brief   Filedescription
 * @author  dg
 * @date    22 May 2026
 ************************************************************/


#ifndef INIT_H_
#define INIT_H_

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
status_t INIT_init_motors(void);
status_t INIT_init_robot(void);
status_t INIT_move_to_start_point(void);
void INIT_SetUnitialized(void);

#endif /* INIT_H_ */
