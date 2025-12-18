/*
 * disk.c
 *
 *  Created on: 18 Dec 2025
 *      Author: dg
 */
#include "disk.h"
 FATFS fs;

status_t DISK_Init(void){

    FRESULT res;

    /* Mount filesystem */
    res = f_mount(&fs, DISK_SD_PATH, 1);
    if (res != FR_OK)
        return kStatus_Fail;

    return kStatus_Success;
}
