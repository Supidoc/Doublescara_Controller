/*
 * log.h
 *
 *  Created on: 18 Dec 2025
 *      Author: dg
 */

#ifndef LOG_H_
#define LOG_H_

#include "fsl_common.h"
#include "FreeRTOS.h"

#define LOG_MAX_MESSAGE_SIZE 100
#define LOG_QUEUE_SIZE 10

typedef enum _LOG_Level {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
}LOG_Level_t;



status_t LOG_Init(void);

void LOG_Task(void * pvParameters);

void LOG_SendLogMessage(LOG_Level_t level, char* message);
BaseType_t LOG_LogCommand(char * pcWriteBuffer, size_t xWriteBufferLen, const char * pcCommandString);

#define LOG_DEBUG(message) LOG_SendLogMessage(LOG_LEVEL_DEBUG, message)
#define LOG_INFO(message) LOG_SendLogMessage(LOG_LEVEL_INFO, message)
#define LOG_WARN(message) LOG_SendLogMessage(LOG_LEVEL_WARN, message)
#define LOG_ERROR(message) LOG_SendLogMessage(LOG_LEVEL_ERROR, message)
#define LOG_FATAL(message) LOG_SendLogMessage(LOG_LEVEL_FATAL, message)

#endif /* LOG_H_ */
