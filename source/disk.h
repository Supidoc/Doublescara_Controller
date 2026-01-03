/************************************************************
 * @file    disk.h
 * @brief   Module for setup of SD card
 *
 * This module provides functionality for initializing and managing the SD card.
 * It uses the FATFS library to mount the filesystem and provides the necessary
 * setup for file operations.
 *
 * @note    Ensure the SD card is properly connected and accessible before
 *          calling the initialization function.
 *
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/


#ifndef DISK_H_
#define DISK_H_

/********************
 *     Includes		*
 ********************/

#include "ff.h"
#include "diskio.h"
#include "fsl_common.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/
/**
 * @brief Converts a macro value to a string.
 */
#define STRINGIFY(x) #x

/**
 * @brief Converts a macro value to a string (wrapper for STRINGIFY).
 */
#define TOSTRING(x) STRINGIFY(x)

/**
 * @brief Path to the SD card disk.
 */
#define DISK_SD_PATH TOSTRING(SDSPIDISK) ":/"

/***************************
 *     Public Typedefs	   *
 ***************************/

/****************************
 *     Public Variables     *
 ****************************/

/**
 * @brief Filesystem object for managing the mounted filesystem.
 *
 * This variable is used by the FATFS library to manage the state of the
 * filesystem, including file operations and disk access.
 *
 * @note This variable must be initialized (by calling DISK_Init) before
 *       performing any file operations.
 */
extern FATFS fs;


/**************************************
 *     Public Function Prototypes	  *
 **************************************/

/**
 * @brief Initializes the disk and mounts the filesystem.
 *
 * This function mounts the filesystem on the specified disk path. It must be called
 * before performing any file operations on the disk.
 *
 * @return kStatus_Success if the filesystem is successfully mounted.
 *         kStatus_Fail if the mounting operation fails.
 */
status_t DISK_Init(void);

#endif /* DISK_H_ */
