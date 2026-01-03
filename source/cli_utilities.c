/************************************************************
 * @file    cli_utilities.c
 * @brief   Implementation file for module
 * @author  dg
 * @date    18 Dec 2025
 ************************************************************/

/********************
 *     Includes		*
 ********************/

#include "cli_utilities.h"
#include "string.h"

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

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t CLU_GetParameterValueString(char * parameterIdentifier, const char * pcCommandString, uint8_t * parameterFound, const char ** pcParameter,
        BaseType_t * parameterStringLength)
{
    size_t i = 1;
    do
    {
        *pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, i, parameterStringLength);
        if (*pcParameter == NULL)
        {
            *parameterStringLength = 0;
            *parameterFound = 0;
            return kStatus_Success;
        }
        size_t cmp = strncmp(parameterIdentifier, *pcParameter, *parameterStringLength);
        if (cmp == 0)
        {
            *pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, i + 1, parameterStringLength);
            if (*pcParameter == NULL)
            {
                *parameterFound = 1;
                *parameterStringLength = 0;
                return kStatus_Success;
            }
            else if (**pcParameter == '-')
            {
                *parameterStringLength = 0;
                *pcParameter = NULL;
                *parameterFound = 1;
                return kStatus_Success;
            }
            else {
                *parameterFound = 1;
                return kStatus_Success;
            }
        }
        i++;
    } while (*pcParameter != NULL);
    return kStatus_Fail;
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
