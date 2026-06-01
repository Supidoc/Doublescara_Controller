/************************************************************
 * @file    disk.c
 * @brief   Implementation file for disk module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include <infrastructure/disk.h>
#include "SEGGER_RTT.h"

/************************************
 *     Private Macros / Defines		*
 ************************************/

/***************************
 *     Private Typedefs	   *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

/****************************
 *     Public Variables     *
 ****************************/

FATFS fs;

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t DISK_init(void)
{

    FRESULT res;
//    MKFS_PARM opt = {
//        .fmt = FM_ANY,
//        .n_fat = 0,
//        .align = 0,
//        .n_root = 0,
//        .au_size = 0,
//    };
//
//    /* Format the volume on startup so the filesystem begins from a known state. */
//    res = f_mkfs(DISK_SD_PATH, &opt, NULL, 0);
//    if (res != FR_OK)
//    {
//        return kStatus_Fail;
//    }

    /* Mount filesystem */
    res = f_mount(&fs, DISK_SD_PATH, 1);
    if (res != FR_OK)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
