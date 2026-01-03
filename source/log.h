/************************************************************
 * @file    log.h
 * @brief   Module for logging data to an SD card
 *
 * This module provides functionality for logging messages to an SD card.
 * It supports multiple log levels, task-safe message queuing, and session-based
 * log file management. The log files are stored in a specific format and can
 * be retrieved using CLI commands.
 *
 * @note    Ensure the disk module is initialized before using this module.
 * @author  dgrob
 * @date    3 Jan 2026
 ************************************************************/


#ifndef LOG_H_
#define LOG_H_

/********************
 *     Includes		*
 ********************/

#include "fsl_common.h"
#include "FreeRTOS.h"

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
#define LOG_QUEUE_SIZE 10


/**
 * @brief Macros for sending log messages at different log levels.
 */
#define LOG_DEBUG(message) LOG_SendLogMessage(LOG_LEVEL_DEBUG, message)
#define LOG_INFO(message) LOG_SendLogMessage(LOG_LEVEL_INFO, message)
#define LOG_WARN(message) LOG_SendLogMessage(LOG_LEVEL_WARN, message)
#define LOG_ERROR(message) LOG_SendLogMessage(LOG_LEVEL_ERROR, message)
#define LOG_FATAL(message) LOG_SendLogMessage(LOG_LEVEL_FATAL, message)

/***************************
 *     Public Typedefs	   *
 ***************************/

/**
 * @brief Enumeration of log levels.
 */
typedef enum _LOG_Level {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
}LOG_Level_t;

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
 * @return kStatus_Success if the initialization is successful.
 *         kStatus_Fail if the initialization fails.
 */
status_t LOG_Init(void);

/**
 * @brief Task function for processing log messages.
 *
 * This function is intended to be run as a FreeRTOS task. It continuously
 * processes log messages from the queue and writes them to the log file.
 *
 * @param[in] pvParameters Pointer to task parameters (not used in this implementation).
 */
void LOG_Task(void * pvParameters);

/**
 * @brief Sends a log message to the log queue.
 *
 * This function creates a log message with the specified log level and
 * message content, and adds it to the log queue. The message will be
 * processed and written to the log file by the logging task.
 *
 * @param[in] level The log level of the message.
 * @param[in] message The content of the log message.
 *
 * @return kStatus_Success if the message is successfully added to the queue.
 *         kStatus_Fail if the queue is full or an error occurs.
 */
status_t LOG_SendLogMessage(LOG_Level_t level, char * message);

/**
 * @brief CLI command for retrieving log files.
 *
 * This function implements a CLI command that retrieves the log file for
 * a specified session ID. If no session ID is provided, the current session's
 * log file is used. The log file is read and its contents are output to the
 * CLI.
 *
 * @param[out] pcWriteBuffer Buffer to store the command output.
 * @param[in] xWriteBufferLen Length of the write buffer.
 * @param[in] pcCommandString The command string entered by the user.
 *
 * @return pdTRUE if there is more data to output, pdFALSE otherwise.
 */
BaseType_t LOG_LogCommand(char * pcWriteBuffer, size_t xWriteBufferLen, const char * pcCommandString);



#endif /* LOG_H_ */
