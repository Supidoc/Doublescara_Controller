/*
 * statemachine_helpers.c
 *
 *  Created on: 18.04.2026
 *      Author: BSc
 */


#include "statemachine_helpers.h"
#include "led.h"
#include "pin_mux.h"
#include "uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


bool ExecuteCmd(CommandPackage pkg){
  switch(pkg.cmd){
      case INIT:
        break;
          case LED_ON:
        break;
          case LED_OFF:
            break;
          case GOTO_XY_ROTATE:
            break;
          case MAGNET_UP:
            break;
          case MAGNET_DOWN:
            break;
          case Z_UP:
            break;
          case Z_DOWN:
            break;
          case ERR:
            return false;
          case FINISH:
            // __das__ __ganze__ halt
            break;
          default:
            return false;
  }
      return true;
}

/**
 * Reads a command from LPUART0 and validates it.
 * Blocks until a valid command (1-10) is received.
 *
 * @__param__[out] __cmd__
 *   pointer to store the received command byte
 * @__param__[out] transID
 *   pointer to store transaction ID (not used in this implementation)
 *
 * @return
 *   TRUE if a valid command was read, FALSE otherwise
 */
bool ReadCmd(CommandPackage *pkg, const char *receivedMessage){

  size_t length = strlen(receivedMessage);

  // convert into 5 fields: __cmd__,x,y,rotate,__transid__
  char *fields[5];
  int   fieldCount = 0;
  char *token = strtok(receivedMessage, ",");

  while (token != NULL && fieldCount < 5) {
    fields[fieldCount++] = token;
      token = strtok(NULL, ",");
  }

  if (fieldCount != 5) {
      return false;
      }

  // parse fields
  pkg->cmd    = (uint8_t)atoi(fields[0]);
  pkg->x      = atof(fields[1]);
  pkg->y      = atof(fields[2]);
  pkg->rotate = atof(fields[3]);
  pkg->transID = (uint8_t)atoi(fields[4]);

  if (uart2IsValidCommand(pkg->cmd))
      return true;
  else {
    return false;
  }
}

/**
 * Sends an OK response via LPUART0.
 *
 * @__param__[in] transID
 *   transaction ID
 */
void SendOK(uint8_t transID)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "OK,%d", transID);
  uart2WriteLine(buf);
}

/**
 * Sends an ERROR response via LPUART0.
 *
 * @__param__[in] transID
 *   transaction ID
 */
void SendError(uint8_t transID)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "ERROR,%d", transID);
  uart2WriteLine(buf);
}

/**
 * Helper for showing __cmd__ string on display instead of __int__.
 *
 * @__param__[in] __cmd__
 *    command
 */
const char* CommandToString(uint8_t cmd) {
  switch (cmd) {
      case INIT:           return "INIT";
      case LED_ON:         return "LED_ON";
      case LED_OFF:        return "LED_OFF";
      case GOTO_XY_ROTATE: return "GOTO_XY_ROTATE";
      case Z_UP:           return "Z_UP";
      case Z_DOWN:         return "Z_DOWN";
      case MAGNET_UP:      return "MAGNET_UP";
      case MAGNET_DOWN:    return "MAGNET_DOWN";
      case ERR:            return "ERROR";
      case FINISH:         return "FINISH";
      case GRAB: return "GRAB";
      case RELEASE: return "RESLEASE";
      default:             return "UNKNOWN";
  }
}
