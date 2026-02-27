/************************************************************
 * @file    log.c
 * @brief   Implementation file for logging module
 * @author  dg
 * @date    3 Jan 2026
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "cli.h"
#include "cli_utilities.h"
#include "disk.h"
#include "peripherals.h"
#include "queue.h"
#include "stdio.h"
#include <log.h>

/************************************
 *     Private Macros / Defines		*
 ************************************/

#define LOG_MAGIC 0x4C4F4746 // Magic Number to identify fileformat ASCII "LOGF"

#define LOG_MAX_LEVEL_STRING_SIZE 5

// Example Log Message: [INFO]:[MC_Task]:[t+113210123]: message \r\n\0
#define LOG_MAX_LOG_LINE_SIZE                                                                      \
    (1 + 5 + 3 + configMAX_TASK_NAME_LEN + 5 + 20 + 2 + LOG_MAX_MESSAGE_SIZE + 2)

/***************************
 *     Private Typedefs	   *
 ***************************/

typedef struct _LOG_QueueItem
{
    TickType_t  tick;
    LOG_Level_t level;
    char        taskName[configMAX_TASK_NAME_LEN];
    char        msg[LOG_MAX_MESSAGE_SIZE];
    uint8_t     silent;
} LOG_QueueItem_t;

typedef struct _LOG_LogHeaderPrefix
{
    uint32_t magic;      // 'LOGF'
    uint16_t version;    // format version
    uint16_t headerSize; // total header size in bytes
} LOG_LogHeaderPrefix_t;

typedef struct _LOG_LogHeader
{
    uint32_t magic;   // 'LOGF'
    uint16_t version; // Format version
    uint16_t headerSize;

    uint32_t sessionId;
    uint32_t tickFreqHz;
} LOG_LogHeader_t;

/*****************************************
 *     Private Function Declarations     *
 *****************************************/

static void       assemble_log_string(char* dest, LOG_QueueItem_t queueItem);
static status_t   append_to_file(uint8_t* buffer, size_t bufferLength);
static void       process(void);
static status_t   set_session_id(void);
static void       assemble_header_string(char* dest, LOG_LogHeader_t header);
static BaseType_t log_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString);
static BaseType_t log_2_cli_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

FIL logFile;

QueueHandle_t logQueue = NULL;

static uint32_t sessionId;

static char LogFilePath[11];

static const CLI_Command_Definition_t xLogCommandDefinition = {
    .pcCommand = "log",
    .pcHelpString =
        "log [OPTIONS]: gets the log for the specified sessionId \r\n \t -s [sessionId] if no "
        "sessionId is given the current sessions log will be used\r\n",
    .pxCommandInterpreter        = log_command,
    .cExpectedNumberOfParameters = -1};
static const CLI_Command_Definition_t xLog2CLICommandDefinition = {
    .pcCommand            = "log2cli",
    .pcHelpString         = "log2cli [LEVEL]: Sets the log level which will be outputted to the "
                            "console.\r\n\t Choose from: NONE,DEBUG,INFO,WARN,ERROR,FATAL\r\n",
    .pxCommandInterpreter = log_2_cli_command,
    .cExpectedNumberOfParameters = -1};

static uint8_t     enableLogToConsole = 1;
static LOG_Level_t consoleLogLevel    = LOG_LEVEL_DEBUG;
static char        consoleOutputString[CLI_MAX_OUTPUT_LENGTH];

/*********************************************
 *      Public Function Implementations		 *
 *********************************************/

status_t LOG_init(void)
{
    logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LOG_QueueItem_t));
    if (logQueue == NULL)
        return kStatus_Fail;

    if (set_session_id() != kStatus_Success)
        return kStatus_Fail;

    // set set file path and name
    sprintf(LogFilePath, DISK_SD_PATH "%lu", sessionId);

    // create log file
    FRESULT res;
    res = f_open(&logFile, LogFilePath, FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK)
        return kStatus_Fail;

    f_close(&logFile);

    LOG_LogHeader_t logHeader;
    logHeader.magic      = LOG_MAGIC;
    logHeader.headerSize = sizeof(LOG_LogHeader_t);
    logHeader.version    = 1;
    logHeader.sessionId  = sessionId;
    logHeader.tickFreqHz = configTICK_RATE_HZ;

    if (append_to_file((uint8_t*)&logHeader, sizeof(LOG_LogHeader_t)) != kStatus_Success)
        return kStatus_Fail;

    return kStatus_Success;
}

status_t LOG_send_log_message(LOG_Level_t level, char* message, uint8_t silent)
{
    if (message == NULL)
        return kStatus_Fail;
    LOG_QueueItem_t queueItem;

    queueItem.level  = level;
    queueItem.silent = silent && 0b1;
    strcpy(queueItem.taskName, pcTaskGetName(NULL));
    queueItem.tick = xTaskGetTickCount();
    strncpy(queueItem.msg, message, LOG_MAX_MESSAGE_SIZE - 1);
    queueItem.msg[LOG_MAX_MESSAGE_SIZE - 1] = '\0';

    if (xQueueSend(logQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
        return kStatus_Fail;
    return kStatus_Success;
}

void LOG_task(void* pvParameters)
{
    LOG_INFO("Started LOG Task");

    CLI_register_command(&xLogCommandDefinition);
    CLI_register_command(&xLog2CLICommandDefinition);
    for (;;)
    {
        process();
        vTaskDelay(1);
    }
}

/**********************************************
 *      Private Function Implementations	  *
 **********************************************/

static BaseType_t log_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString)
{
    static uint8_t         fileOpened = 0;
    static uint32_t        fileSessionId;
    FRESULT                res;
    LOG_LogHeaderPrefix_t  prefix;
    static LOG_LogHeader_t header;
    UINT                   br;
    static char            localLogFilePath[11];
    static FIL             localLogFile;

    if (fileOpened != 1u)
    {
        uint8_t     parameterFound;
        const char* pcParameter = NULL;
        BaseType_t  parameterStringLength;
        CLU_get_parameter_value_string("-s", pcCommandString, &parameterFound, &pcParameter,
                                       &parameterStringLength);
        if (parameterFound == 0 || parameterStringLength == 0)
        {
            fileSessionId = sessionId;
        }
        else
        {
            fileSessionId = strtoul(pcParameter, NULL, 0);
        }
        sprintf(localLogFilePath, DISK_SD_PATH "%lu", fileSessionId);

        res = f_open(&localLogFile, localLogFilePath, FA_READ | FA_OPEN_ALWAYS);
        if (res != FR_OK)
            return kStatus_Fail;

        res = f_read(&localLogFile, &prefix, sizeof(prefix), &br);
        if (res != FR_OK || br != sizeof(prefix))
        {
            strncpy(pcWriteBuffer, "Invalid file format", strlen("Invalid file format") + 1);
            return pdFALSE;
        }

        if (prefix.magic != LOG_MAGIC)
        {
            strncpy(pcWriteBuffer, "Invalid file format", strlen("Invalid file format") + 1);
            return pdFALSE;
        }

        f_rewind(&localLogFile);
        res = f_read(&localLogFile, &header, sizeof(header), &br);
        if (res != FR_OK || br != sizeof(header))
        {
            strncpy(pcWriteBuffer, "Invalid file format", strlen("Invalid file format") + 1);
            return pdFALSE;
        }

        assemble_header_string(pcWriteBuffer, header);

        fileOpened = 1u;
    }
    else
    {

        LOG_QueueItem_t queueItem;
        res = f_read(&localLogFile, &queueItem, sizeof(LOG_QueueItem_t), &br);

        if (br == sizeof(LOG_QueueItem_t))
        {
            assemble_log_string(pcWriteBuffer, queueItem);
        }
    }
    if (f_eof(&localLogFile) == 0)
    {
        return pdTRUE;
    }

    f_close(&localLogFile);
    fileOpened = 0u;
    return pdFALSE;
}

static BaseType_t log_2_cli_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString)
{
    uint8_t     parameterFound;
    const char* pcParameter = NULL;
    BaseType_t  parameterStringLength;
    CLU_get_parameter_value_string("NONE", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 0;
        return pdFALSE;
    }
    CLU_get_parameter_value_string("DEBUG", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 1;
        consoleLogLevel    = LOG_LEVEL_DEBUG;
        return pdFALSE;
    }
    CLU_get_parameter_value_string("INFO", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 1;
        consoleLogLevel    = LOG_LEVEL_INFO;
        return pdFALSE;
    }
    CLU_get_parameter_value_string("WARN", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 1;
        consoleLogLevel    = LOG_LEVEL_WARN;
        return pdFALSE;
    }
    CLU_get_parameter_value_string("ERROR", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 1;
        consoleLogLevel    = LOG_LEVEL_ERROR;
        return pdFALSE;
    }
    CLU_get_parameter_value_string("FATAL", pcCommandString, &parameterFound, &pcParameter,
                                   &parameterStringLength);
    if (parameterFound == 1)
    {
        enableLogToConsole = 1;
        consoleLogLevel    = LOG_LEVEL_FATAL;
        return pdFALSE;
    }
    strncpy(pcWriteBuffer, "Invalid Log level", strlen("Invalid Log level") + 1);
    return pdFALSE;
}

static void assemble_log_string(char* dest, LOG_QueueItem_t queueItem)
{

    char* levelString;
    switch (queueItem.level)
    {
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

    uint32_t timestampMs = pdTICKS_TO_MS(queueItem.tick);
    char     timestampString[21];
    sprintf(timestampString, "%lu", (uint32_t)timestampMs);
    strcat(dest, timestampString);

    strcat(dest, "]:");

    strcat(dest, queueItem.msg);

    strcat(dest, " \r\n\0");
}

static void assemble_header_string(char* dest, LOG_LogHeader_t header)
{

    strcpy(dest, "Session: ");

    char sessionIdString[11];
    sprintf(sessionIdString, "%lu", header.sessionId);
    strcat(dest, sessionIdString);

    strcat(dest, " Version: ");

    char versionString[6];
    sprintf(versionString, "%d", header.version);
    strcat(dest, versionString);

    strcat(dest, " \r\n\0");
}

static status_t append_to_file(uint8_t* buffer, size_t bufferLength)
{
    FRESULT res;
    res = f_open(&logFile, LogFilePath, FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK)
        return kStatus_Fail;

    res = f_lseek(&logFile, f_size(&logFile));
    if (res != FR_OK)
        return kStatus_Fail;

    UINT bw;
    res = f_write(&logFile, buffer, bufferLength, &bw);
    if (res != FR_OK || bw != bufferLength)
        return kStatus_Fail;

    res = f_sync(&logFile);
    if (res != FR_OK)
        return kStatus_Fail;

    res = f_close(&logFile);
    if (res != FR_OK)
        return kStatus_Fail;

    return kStatus_Success;
}

static void process(void)
{
    LOG_QueueItem_t queueItem;
    BaseType_t      status;
    status = xQueueReceive(logQueue, &queueItem, portMAX_DELAY);
    if (status != pdPASS)
        return;

    if (enableLogToConsole && queueItem.level > consoleLogLevel && queueItem.silent == 0)
    {
        assemble_log_string(consoleOutputString, queueItem);
        size_t len = strlen((char*)consoleOutputString);
        LPUART_RTOS_Send(&LPUART0_rtos_handle, (uint8_t*)consoleOutputString, len);
    }
    append_to_file((uint8_t*)&queueItem, sizeof(queueItem));
}

static status_t set_session_id(void)
{
    FRESULT res;
    FIL     file;
    res = f_open(&file, DISK_SD_PATH "session.id", FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK)

        return kStatus_Fail;

    UINT br;
    f_read(&file, &sessionId, sizeof(sessionId), &br);
    f_rewind(&file);
    if (br != sizeof(sessionId))
    {
        sessionId = 1;
    }
    else
    {
        sessionId++;
    }
    UINT bw;
    res = f_write(&file, &sessionId, sizeof(sessionId), &bw);
    if (res != FR_OK)
        return kStatus_Fail;
    f_sync(&file);

    f_close(&file);
    return kStatus_Success;
}
