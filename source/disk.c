/************************************************************
 * @file    disk.c
 * @brief   Implementation file for disk module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include "disk.h"

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

    /* Mount filesystem */
    res = f_mount(&fs, DISK_SD_PATH, 1);
    if (res != FR_OK)
        return kStatus_Fail;

    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
