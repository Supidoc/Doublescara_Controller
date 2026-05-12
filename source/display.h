/************************************************************
 * @file    display.h
 * @brief   I2C display interface
 * @author  rp
 * @date    12 Apr 2026
 ************************************************************/

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "fsl_common.h"

status_t DISPLAY_init(void);
status_t DISPLAY_write(const char *text);
void DISPLAY_task(void *pvParameters);

#endif /* DISPLAY_H_ */

