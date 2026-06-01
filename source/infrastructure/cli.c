/************************************************************
 * @file    cli.c
 * @brief   Implementation file for CLI module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include <infrastructure/cli.h>
#include <infrastructure/log.h>
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "peripherals.h"
#include "stdio.h"
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

static void       process(void);
static BaseType_t x_ping_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static SemaphoreHandle_t cliMutex = NULL;

static const CLI_Command_Definition_t xPingCommandDefinition = {
    .pcCommand                   = "ping",
    .pcHelpString                = "ping: Returns Pong\r\n",
    .pxCommandInterpreter        = x_ping_command,
    .cExpectedNumberOfParameters = 0};

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t CLI_init(void)
{
    BaseType_t status;

    cliMutex = xSemaphoreCreateMutex();
    if (cliMutex == NULL)
    {
        return kStatus_Fail;
    }

    status = CLI_register_command(&xPingCommandDefinition);
    if (status != kStatus_Success)
    {
        return kStatus_Fail;
    }
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    SEGGER_RTT_WriteString(0, RTT_CTRL_CLEAR);
    return kStatus_Success;
}

status_t CLI_register_command(const CLI_Command_Definition_t* const pxCommandToRegister)
{
    BaseType_t status;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        if (xSemaphoreTake(cliMutex, portMAX_DELAY) != pdTRUE)
        {
            LOG_ERROR("Failed to acquire CLI mutex for command registration");
            return kStatus_Fail;
        }
    }

    status = FreeRTOS_CLIRegisterCommand(pxCommandToRegister);
    if (status != pdTRUE)
    {
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            LOG_ERROR("Failed to register CLI command");
            xSemaphoreGive(cliMutex);
        }
        return kStatus_Fail;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        char logMsg[60];
        snprintf(logMsg, sizeof(logMsg), "Registered CLI command: %s",
                 pxCommandToRegister->pcCommand);
        LOG_DEBUG(logMsg);
        xSemaphoreGive(cliMutex);
    }

    return kStatus_Success;
}

void CLI_task(void* pvParameters)
{
    LOG_INFO("Started CLI Task");
    for (;;)
    {
        process();
        vTaskDelay(1);
    }
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/

static void process(void)
{

    uint8_t        cRxedChar;
    static uint8_t cInputIndex = 0;
    size_t         rxReceivedCount;
    BaseType_t     xMoreDataToFollow;

    /* The input and output buffers are declared static to keep them off the stack. */
    static uint8_t pcOutputString[CLI_MAX_OUTPUT_LENGTH];
    static uint8_t pcInputString[CLI_MAX_INPUT_LENGTH];

    /* This implementation reads a single character at a time. Wait in the

     Blocked state until a character is received. */
#if CLI_USE_RTT
    if (SEGGER_RTT_HasKey() == 0)
    {
        return;
    }
    cRxedChar = SEGGER_RTT_GetKey();
#else
    LPUART_RTOS_Receive(&LPUART0_rtos_handle, &cRxedChar, 1, &rxReceivedCount);

    if (rxReceivedCount == 0)
    {
        return;
    }
#endif
    if (cRxedChar == '\n')

    {
        if (xSemaphoreTake(cliMutex, portMAX_DELAY) != pdTRUE)
        {
            LOG_ERROR("Failed to acquire CLI mutex for command processing");
            return;
        }
        /* A newline character was received, so the input command string is
         complete and can be processed. Transmit a line separator, just to
         make the output easier to read. */
#if CLI_USE_RTT
        SEGGER_RTT_Write(0, "\r\n", strlen("\r\n"));
#else
        LPUART_RTOS_Send(&LPUART0_rtos_handle, (unsigned char*)"\r\n", strlen("\r\n"));
#endif
        if (strncmp((char*)pcInputString, "log", strlen("log")) != 0)
        {
            snprintf((char*)pcOutputString, sizeof(pcOutputString), "USER: %s", pcInputString);
#if CLI_SHOW_COMMAND_INPUT

            LOG_INFO((char*)pcOutputString);

#else
            LOG_INFO_SILENT((char*)pcOutputString);
#endif
            memset(pcOutputString, 0x00, CLI_MAX_OUTPUT_LENGTH);
        }
        /* The command interpreter is called repeatedly until it returns
         pdFALSE. See the "Implementing a command" documentation for an
         exaplanation of why this is. */
        do

        {

            /* Send the command string to the command interpreter. Any
             output generated by the command interpreter will be placed in the
             pcOutputString buffer. */

            xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                (char*)pcInputString,  /* The command string.*/
                (char*)pcOutputString, /* The output buffer. */
                CLI_MAX_OUTPUT_LENGTH  /* The size of the output buffer. */
            );

            /* Write the output generated by the command interpreter to the
             console. */
#if CLI_USE_RTT
            // Exclude log and help command from printing info prefix
            if (strncmp((char*)pcInputString, "help", strlen("help")) == 0 ||
                strncmp((char*)pcInputString, "log", strlen("log")) == 0)
            {
                SEGGER_RTT_Write(0, pcOutputString, strlen(pcOutputString));
            }
            else
            {
                LOG_INFO(pcOutputString);
            }
#else
            LPUART_RTOS_Send(&LPUART0_rtos_handle, pcOutputString, strlen((char*)pcOutputString));
#endif
        } while (xMoreDataToFollow != pdFALSE);

        /* All the strings generated by the input command have been sent.
         Processing of the command is complete. Clear the input string ready
         to receive the next command. */

        cInputIndex = 0;
        memset(pcInputString, 0x00, CLI_MAX_INPUT_LENGTH);

        xSemaphoreGive(cliMutex);
    }
    else
    {
        /* The if() clause performs the processing after a newline character
         is received. This else clause performs the processing if any other
         character is received. */

        if (cRxedChar == '\r')

        {
            /* Ignore carriage returns. */
        }
        else if (cRxedChar == '\0')

        {
            /* Ignore null termination. */
        }
        else if (cRxedChar == '\b')
        {
            /* Backspace was pressed. Erase the last character in the input
             buffer - if there are any. */
            if (cInputIndex > 0)
            {
                cInputIndex--;
                pcInputString[cInputIndex] = '\0';
            }
        }
        else
        {
            /* A character was entered. It was not a new line, backspace
             or carriage return, so it is accepted as part of the input and
             placed into the input buffer. When a n is entered the complete
             string will be passed to the command interpreter. */
            if (cInputIndex < CLI_MAX_INPUT_LENGTH)
            {
                pcInputString[cInputIndex] = cRxedChar;
                cInputIndex++;
            }
        }
    }
}

static BaseType_t x_ping_command(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString)
{
    strncpy(pcWriteBuffer, "Pong", strlen("Pong") + 1);
    return pdFALSE;
}
