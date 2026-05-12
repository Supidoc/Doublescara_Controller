/*
 * statemachine_helpers.h
 *
 *  Created on: 18.04.2026
 *      Author: BSc
 */

#ifndef STATEMACHINE_HELPERS_H_
#define STATEMACHINE_HELPERS_H_

#include <stdint.h>
#include <stdbool.h>

/* ── Kommando-IDs ──────────────────────────────────────────────────── */

typedef enum
{
    INIT           = 1,
    LED_ON         = 2,
    LED_OFF        = 3,
    GOTO_XY_ROTATE = 4,
    Z_UP           = 5,
    Z_DOWN         = 6,
    MAGNET_UP      = 7,
    MAGNET_DOWN    = 8,
    FINISH         = 9,
	GRAB = 10,
	RELEASE = 11,
    ERR            = 255
} Command;


/* ── Kommandopaket ─────────────────────────────────────────────────── */

typedef struct
{
    Command cmd;
    float   x;
    float   y;
    float   rotate;
    uint8_t transID;
} CommandPackage;


/**
 * @brief  Führt das Kommando im pkg aus.
 * @return true bei Erfolg, false bei unbekanntem oder fehlgeschlagenem Kommando.
 */
bool ExecuteCmd(CommandPackage pkg);


/**
 * @brief  Liest und parst die nächste Zeile aus dem UART-Puffer.
 *         Format: "cmd,x,y,rotate,transID"
 * @param  pkg  Zeiger auf das zu befüllende CommandPackage.
 * @param  receivedMessage  Zeiger auf den empfangenen String (Nullterminiert).
 * @return true bei gültigem Kommando, false bei Parse-Fehler oder unbekanntem Kommando.
 *
 */
bool ReadCmd(CommandPackage *pkg, const char *receivedMessage);

/**
 * @brief  Sendet "OK,<transID>" über LPUART0.
 */
void SendOK(uint8_t transID);

/**
 * @brief  Sendet "ERROR,<transID>" über LPUART0.
 */
void SendError(uint8_t transID);

/**
 * @brief  Gibt den Namen eines Kommandos als String zurück (für Display/Log).
 */
const char* CommandToString(uint8_t cmd);

#endif /* STATEMACHINE_HELPERS_H_ */
