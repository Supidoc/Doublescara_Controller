/************************************************************
 * @file    log.c
 * @brief   Implementation file for module
 * @author  dgrob
 * @date    3 Jan 2026
 ************************************************************/

/********************
 *     Includes		*
 ********************/

#include <log.h>
#include "peripherals.h"
#include "disk.h"
#include "stdio.h"
#include "queue.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "cli_utilities.h"

/************************************
 *     Private Macros / Defines		*
 ************************************/

#define LOG_MAGIC 0x4C4F4746  // Magic Number to identify fileformat ASCII "LOGF"

#define LOG_MAX_LEVEL_STRING_SIZE 5

// Example Log Message: [INFO]:[MC_Task]:[t+113210123]: message \r\n\0
#define LOG_MAX_LOG_LINE_SIZE (1+5+3+configMAX_TASK_NAME_LEN+5+20+2+LOG_MAX_MESSAGE_SIZE +2)

/***************************
 *     Private Typedefs	   *
 ***************************/

typedef struct _LOG_QueueItem {
	TickType_t tick;
	LOG_Level_t level;
	char taskName[configMAX_TASK_NAME_LEN];
	char msg[LOG_MAX_MESSAGE_SIZE];
} LOG_QueueItem_t;

typedef struct _LOG_LogHeaderPrefix {
	uint32_t magic;       // 'LOGF'
	uint16_t version;     // format version
	uint16_t headerSize;  // total header size in bytes
} LOG_LogHeaderPrefix_t;

typedef struct _LOG_LogHeader {
	uint32_t magic;        // 'LOGF'
	uint16_t version;      // Format version
	uint16_t headerSize;

	uint32_t sessionId;
	uint32_t tickFreqHz;
} LOG_LogHeader_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static void LOG_AssembleLogString(char *dest, LOG_QueueItem_t queueItem);
static status_t LOG_AppendToFile(uint8_t *buffer, size_t bufferLength);
static void LOG_Process(void);
static status_t LOG_SetSessionId(void);
static void LOG_AssembleHeaderString(char *dest, LOG_LogHeader_t header);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

FIL logFile;

QueueHandle_t logQueue = NULL;

static char LogMessageBuffer[LOG_MAX_LOG_LINE_SIZE];

static uint32_t sessionId;

static char LogFilePath[11];

static const CLI_Command_Definition_t xLogCommandDefinition =
		{ .pcCommand = "log",
				.pcHelpString =
						"log [OPTIONS]: gets the log for the specified sessionId \r\n \t -s [sessionId] if no sessionId is given the current sessions log will be used\r\n",
				.pxCommandInterpreter = LOG_LogCommand,
				.cExpectedNumberOfParameters = -1 };

/*********************************************
 *      Public Function Implementations		 *
 *********************************************/

status_t LOG_Init(void) {
	logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LOG_QueueItem_t));
	if (logQueue == NULL)
		return kStatus_Fail;

	LOG_SetSessionId();

	//set set file path and name
	sprintf(LogFilePath, DISK_SD_PATH "%u", sessionId);

	//create log file
	FRESULT res;
	res = f_open(&logFile, LogFilePath,
	FA_WRITE | FA_OPEN_ALWAYS);
	if (res != FR_OK)
		return kStatus_Fail;

	f_close(&logFile);

	LOG_LogHeader_t logHeader;
	logHeader.magic = LOG_MAGIC;
	logHeader.headerSize = sizeof(LOG_LogHeader_t);
	logHeader.version = 1;
	logHeader.sessionId = sessionId;
	logHeader.tickFreqHz = configTICK_RATE_HZ;

	LOG_AppendToFile((uint8_t*) &logHeader, sizeof(LOG_LogHeader_t));

	return kStatus_Success;
}

status_t LOG_SendLogMessage(LOG_Level_t level, char *message) {
	LOG_QueueItem_t queueItem;

	queueItem.level = level;
	strcpy(queueItem.taskName, pcTaskGetName(NULL));
	queueItem.tick = xTaskGetTickCount();
	strncpy(queueItem.msg, message, LOG_MAX_MESSAGE_SIZE - 1);
	queueItem.msg[LOG_MAX_MESSAGE_SIZE - 1] = '\0';

	if (xQueueSend(logQueue, (void* ) &queueItem, portMAX_DELAY) != pdTRUE)
		return kStatus_Fail;
	return kStatus_Success;
}

BaseType_t LOG_LogCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
		const char *pcCommandString) {
	static uint8_t fileOpened = 0;
	static uint32_t fileSessionId;
	FRESULT res;
	LOG_LogHeaderPrefix_t prefix;
	static LOG_LogHeader_t header;
	UINT br;
	static char localLogFilePath[11];
	static FIL localLogFile;

	if (fileOpened != 1u) {
		uint8_t parameterFound;
		const char *pcParameter = NULL;
		BaseType_t parameterStringLength;
		CLU_GetParameterValueString("-s", pcCommandString, &parameterFound,
				&pcParameter, &parameterStringLength);
		if (parameterFound == 0 || parameterStringLength == 0) {
			fileSessionId = sessionId;
		} else {
			fileSessionId = strtoul(pcParameter, NULL, 0);
		}
		sprintf(localLogFilePath, DISK_SD_PATH "%u", fileSessionId);

		res = f_open(&localLogFile, localLogFilePath,
		FA_READ | FA_OPEN_ALWAYS);
		if (res != FR_OK)
			return kStatus_Fail;

		res = f_read(&localLogFile, &prefix, sizeof(prefix), &br);
		if (res != FR_OK || br != sizeof(prefix)) {
			strncpy(pcWriteBuffer, "Invalid file format",
					strlen("Invalid file format") + 1);
			return pdFALSE;
		}

		if (prefix.magic != LOG_MAGIC) {
			strncpy(pcWriteBuffer, "Invalid file format",
					strlen("Invalid file format") + 1);
			return pdFALSE;
		}

		f_rewind(&localLogFile);
		res = f_read(&localLogFile, &header, sizeof(header), &br);
		if (res != FR_OK || br != sizeof(header)) {
			strncpy(pcWriteBuffer, "Invalid file format",
					strlen("Invalid file format") + 1);
			return pdFALSE;
		}

		LOG_AssembleHeaderString(pcWriteBuffer, header);

		fileOpened = 1u;
	} else {

		LOG_QueueItem_t queueItem;
		res = f_read(&localLogFile, &queueItem, sizeof(LOG_QueueItem_t), &br);

		if (br == sizeof(LOG_QueueItem_t)) {
			LOG_AssembleLogString(pcWriteBuffer, queueItem);
		}
	}
	if (f_eof(&localLogFile) == 0) {
		return pdTRUE;
	}

	f_close(&localLogFile);
	fileOpened = 0u;
	return pdFALSE;
}

void LOG_Task(void *pvParameters) {
	LOG_SendLogMessage(LOG_LEVEL_INFO, "Started LOG Task");

	CLI_RegisterCommand(&xLogCommandDefinition);
	for (;;) {
		LOG_Process();
		vTaskDelay(1);
	}
}

/**********************************************
 *      Private Function Implementations	  *
 **********************************************/

static void LOG_AssembleLogString(char *dest, LOG_QueueItem_t queueItem) {

	char *levelString;
	switch (queueItem.level) {
	case LOG_LEVEL_DEBUG:
		levelString = "DEBUG";
		break;
	case LOG_LEVEL_INFO:
		levelString = "INFO";
		break;
	case LOG_LEVEL_WARN:
		levelString = "WARN";
		break;
	case LOG_LEVEL_ERROR:
		levelString = "ERROR";
		break;
	case LOG_LEVEL_FATAL:
		levelString = "FATAL";
		break;

	};

	strcpy(dest, "[");

	strcat(dest, levelString);

	strcat(dest, "]:[");

	strcat(dest, queueItem.taskName);

	strcat(dest, "]:[t+");

	uint64_t timestampMs = pdTICKS_TO_MS(queueItem.tick);
	char timestampString[21];
	sprintf(timestampString, "%llu\0", (uint64_t) timestampMs);
	strcat(dest, timestampString);

	strcat(dest, "]:");

	strcat(dest, queueItem.msg);

	strcat(dest, " \r\n\0");
}

static void LOG_AssembleHeaderString(char *dest, LOG_LogHeader_t header) {

	strcpy(dest, "Session: ");

	char sessionIdString[11];
	sprintf(sessionIdString, "%lu\0", header.sessionId);
	strcat(dest, sessionIdString);

	strcat(dest, " Version: ");

	char versionString[6];
	sprintf(versionString, "%d\0", header.version);
	strcat(dest, versionString);

	strcat(dest, " \r\n\0");
}

static status_t LOG_AppendToFile(uint8_t *buffer, size_t bufferLength) {
	FRESULT res;
	res = f_open(&logFile, LogFilePath,
	FA_WRITE | FA_OPEN_ALWAYS);
	if (res != FR_OK)
		return kStatus_Fail;

	res = f_lseek(&logFile, f_size(&logFile));
	if (res != FR_OK)
		return kStatus_Fail;

	uint32_t bw;
	res = f_write(&logFile, buffer, bufferLength, &bw);
	if (res != FR_OK && bw == bufferLength)
		return kStatus_Fail;

	res = f_sync(&logFile);
	if (res != FR_OK)
		return kStatus_Fail;

	res = f_close(&logFile);
	if (res != FR_OK)
		return kStatus_Fail;

	return kStatus_Success;
}

static void LOG_Process(void) {
	LOG_QueueItem_t queueItem;
	BaseType_t status;
	status = xQueueReceive(logQueue, &queueItem, portMAX_DELAY);
	if (status != pdPASS)
		return;

	//LOG_AssembleLogString(LogMessageBuffer, queueItem);

	//size_t bufferLength = strlen(LogMessageBuffer);
	LOG_AppendToFile((uint8_t*) &queueItem, sizeof(queueItem));
}

static status_t LOG_SetSessionId(void) {
	FRESULT res;
	FIL file;
	res = f_open(&file,
	DISK_SD_PATH"session.id", FA_READ |
	FA_WRITE | FA_OPEN_ALWAYS);
	if (res != FR_OK)
		return kStatus_Fail;

	uint32_t br;
	f_read(&file, &sessionId, sizeof(sessionId), &br);
	f_rewind(&file);
	if (br != sizeof(sessionId)) {
		sessionId = 1;
	} else {
		sessionId++;
	}
	uint32_t bw;
	res = f_write(&file, &sessionId, sizeof(sessionId), &bw);
	if (res != FR_OK)
		return kStatus_Fail;
	f_sync(&file);

	f_close(&file);
	return kStatus_Success;
}
