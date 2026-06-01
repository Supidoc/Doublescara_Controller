/************************************************************
 * @file    disk.h
 * @brief   SD card filesystem setup module.
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

/**
 * @defgroup DISK_Module SD Card Disk Module
 * @brief   Functions for initializing and managing SD card filesystem
 * @{
 */

#ifndef INFRASTRUCTURE_DISK_H_
#define INFRASTRUCTURE_DISK_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes		*
     ********************/

#include "ff.h"
#include "fsl_common.h"
#include "diskio.h"

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
     * before performing any file operations on the disk. This should be one of the first
     * initialization functions called in your application startup sequence.
     *
     * @note This function is NOT task-safe and must be called during system initialization.
     * @note Other modules that depend on disk access (e.g., LOG) require this to be initialized
     * first.
     * @warning File operations are not safe until this function completes successfully.
     * @note Ensure SD card hardware is properly connected and detected before calling this.
     *
     * @return kStatus_Success if the filesystem is successfully mounted.
     *         kStatus_Fail if the mounting operation fails (card not detected or corrupted).
     *
     * @see fs (external filesystem object)
     * @see DISK_SD_PATH
     */
    status_t DISK_init(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INFRASTRUCTURE_DISK_H_ */
