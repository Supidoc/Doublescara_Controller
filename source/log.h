/************************************************************
 * @file    log.h
 * @brief   Module for logging data to an SD card.
 *
 * This module provides functionality for logging messages to an SD card with
 * optimized string assembly using snprintf(). It supports multiple log levels (DEBUG, INFO,
 * WARN, ERROR), task-safe message queuing, and session-based log file management.
 * Log output format: [LEVEL]:[TIMESTAMP]:[MESSAGE]\r\n
 * The log files can be retrieved using CLI commands.
 *
 * @note    Ensure the disk module is initialized before using this module.
 * @note    Console output can be suppressed for silent logging while file logging continues.
 * @author  dg
 * @date    6 Apr 2026
 ************************************************************/

/**
 * @defgroup LOG_Module Logging Module
 * @brief   Functions for logging messages to SD card with multiple log levels
 * @{
 */

#ifndef LOG_H_
#define LOG_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes		*
     ********************/

#include "FreeRTOS.h"
#include "fsl_common.h"

/***********************************
 *     Public Macros / Defines	   *
 ***********************************/

/**
 * @brief Maximum size of a log message (in characters).
 */
#define LOG_MAX_MESSAGE_SIZE 100

/**
 * @brief Size of the log message queue.
 */
#define LOG_QUEUE_SIZE 30

/**
 * @brief Macros for sending log messages at different log levels.
 */
#define LOG_DEBUG(message) LOG_send_log_message(LOG_LEVEL_DEBUG, message, 0)
#define LOG_INFO(message) LOG_send_log_message(LOG_LEVEL_INFO, message, 0)
#define LOG_WARN(message) LOG_send_log_message(LOG_LEVEL_WARN, message, 0)
#define LOG_ERROR(message) LOG_send_log_message(LOG_LEVEL_ERROR, message, 0)
#define LOG_FATAL(message) LOG_send_log_message(LOG_LEVEL_FATAL, message, 0)

/**
 * @brief Macros for sending log messages at different log levels without outputting to console.
 */
#define LOG_DEBUG_SILENT(message) LOG_send_log_message(LOG_LEVEL_DEBUG, message, 1)
#define LOG_INFO_SILENT(message) LOG_send_log_message(LOG_LEVEL_INFO, message, 1)
#define LOG_WARN_SILENT(message) LOG_send_log_message(LOG_LEVEL_WARN, message, 1)
#define LOG_ERROR_SILENT(message) LOG_send_log_message(LOG_LEVEL_ERROR, message, 1)
#define LOG_FATAL_SILENT(message) LOG_send_log_message(LOG_LEVEL_FATAL, message, 1)

    /***************************
     *     Public Typedefs	   *
     ***************************/

    /**
     * @brief Enumeration of log levels.
     */
    typedef enum _LOG_Level
    {
        /** @brief Debug level - detailed diagnostic information */
        LOG_LEVEL_DEBUG,
        /** @brief Info level - general informational messages */
        LOG_LEVEL_INFO,
        /** @brief Warning level - warning messages for potential issues */
        LOG_LEVEL_WARN,
        /** @brief Error level - error messages for failures */
        LOG_LEVEL_ERROR,
        /** @brief Fatal level - critical system failures */
        LOG_LEVEL_FATAL
    } LOG_Level_t;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes	  *
     **************************************/

    /**
     * @brief Initializes the logging module.
     *
     * This function initializes the log message queue, sets up the log file for
     * the current session, and writes the log file header. It must be called
     * before using any other functions in this module.
     *
     * @note Ensure DISK_init() is called before initializing the logging module.
     * @note This function is NOT task-safe and must be called during system initialization.
     * @note The LOG_task() should be started after this function completes successfully.
     *
     * @return kStatus_Success if the initialization is successful.
     *         kStatus_Fail if the initialization fails (disk error or queue creation failed).
     *
     * @see DISK_init()
     * @see LOG_task()
     */
    status_t LOG_init(void);

    /**
     * @brief Task function for processing log messages.
     *
     * This function is intended to be run as a FreeRTOS task. It continuously
     * processes log messages from the queue and writes them to the log file.
     *
     * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
     *
     * @note This task should be started after LOG_init() completes successfully.
     * @note This task is blocked on the message queue and wakes when messages are available.
     * @warning This function does not return; it is expected to run as a perpetual FreeRTOS task.
     *
     * @see LOG_init()
     */
    void LOG_task(void* pvParameters);

    /**
     * @brief Sends a log message to the log queue.
     *
     * This function creates a log message with the specified log level and
     * message content, and adds it to the log queue. The message will be
     * processed and written to the log file by the logging task.
     *
     * @param[in] level The log level of the message.
     * @param[in] message The content of the log message (max 100 characters).
     * @param[in] silent If 1, the message will only be written to the log file. If 0, the message
     *                   will be output to the console in addition to being logged to the file.
     *
     * @note This function is TASK-SAFE and can be called from any task context.
     * @note Messages are queued and processed asynchronously by LOG_task().
     * @warning If the queue is full, the message will be dropped and the function returns
     * kStatus_Fail.
     *
     * @return kStatus_Success if the message is successfully added to the queue.
     *         kStatus_Fail if the queue is full or an error occurs.
     *
     * @see LOG_MAX_MESSAGE_SIZE
     * @see LOG_QUEUE_SIZE
     */
    status_t LOG_send_log_message(LOG_Level_t level, char* message, uint8_t silent);

    /**
     * @brief Waits for the log queue to become empty.
     *
     * This function blocks the calling task until the log queue is completely empty.
     * Multiple tasks can wait simultaneously on the same queue; all waiting tasks
     * will be unblocked when the queue becomes empty.
     *
     * @param[in] xTicksToWait The maximum amount of time to wait, in ticks.
     *                         Use portMAX_DELAY to wait indefinitely.
     *
     * @note This function is TASK-SAFE and can be called from any task context.
     * @note The event bit is automatically cleared after this function returns successfully,
     *       so the next call will wait for the queue to empty again.
     * @warning Calling this function immediately after boot (before LOG_init) will fail.
     *
     * @return kStatus_Success if the queue became empty within the timeout period.
     *         kStatus_Fail if the timeout expired or the logging system is not initialized.
     *
     * @see LOG_init()
     * @see LOG_send_log_message()
     */
    status_t LOG_wait_for_queue_empty(TickType_t xTicksToWait);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* LOG_H_ */
