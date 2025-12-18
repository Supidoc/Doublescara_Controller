/*
 * disk.h
 *
 *  Created on: 18 Dec 2025
 *      Author: dg
 */

#ifndef DISK_H_
#define DISK_H_
#include "ff.h"
#include "diskio.h"
#include "fsl_common.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define DISK_SD_PATH TOSTRING(SDSPIDISK) ":/"

extern FATFS fs;

status_t DISK_Init(void);

#endif /* DISK_H_ */
