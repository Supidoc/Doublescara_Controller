/**
 * @file statemachine_helpers.h
 * @brief Helpers for the application state machine (command parsing & utils)
 *
 * Parsing helpers, command definitions and small utility functions used by
 * the statemachine task.
 */

#ifndef STATEMACHINE_HELPERS_H_
#define STATEMACHINE_HELPERS_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /********************
     *     Includes    *
     ********************/

#include <stdint.h>
#include <stdbool.h>

    /***********************************
     *     Public Macros / Defines     *
     ***********************************/

    /***************************
     *     Public Typedefs     *
     ***************************/

    /**
     * @brief Command identifiers for the state machine.
     */
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
        GRAB           = 10,
        RELEASE        = 11,
        ERR            = 255
    } Command;

    /**
     * @brief Command package structure used to pass parsed commands.
     */
    typedef struct
    {
        Command cmd;     /**< Command identifier */
        float   x;       /**< X coordinate / parameter */
        float   y;       /**< Y coordinate / parameter */
        float   rotate;  /**< Rotation parameter */
        uint8_t transID; /**< Transaction ID for acknowledgements */
    } CommandPackage;

    /****************************
     *     Public Variables     *
     ****************************/

    /**************************************
     *     Public Function Prototypes    *
     **************************************/

    /**
     * @brief Execute the command contained in @p pkg.
     *
     * @param pkg CommandPackage by value
     * @return true on success, false on unknown or failed command
     */
    bool ExecuteCmd(CommandPackage pkg);

    /**
     * @brief Parse a received line into a @ref CommandPackage.
     *
     * Expected format: "cmd,x,y,rotate,transID"
     *
     * @param[out] pkg Pointer to package to fill
     * @param[in] receivedMessage NUL-terminated received string
     * @return true if parsing succeeded and command is known, false otherwise
     */
    bool ReadCmd(CommandPackage* pkg, const char* receivedMessage);

    /**
     * @brief Send an OK acknowledgement for @p transID.
     *
     * Typically sends "OK,<transID>" over the serial interface.
     */
    void SendOK(uint8_t transID);

    /**
     * @brief Send an ERROR acknowledgement for @p transID.
     */
    void SendError(uint8_t transID);

    /**
     * @brief Return a textual name for command @p cmd.
     *
     * Useful for logging or display.
     */
    const char* CommandToString(uint8_t cmd);

    /**
     * @brief Show a short one-line text (display/log helper).
     */
    void Show1Liner(const unsigned char* text0);

    /**
     * @brief Show a short two-line text (display/log helper).
     */
    void Show2Liner(const unsigned char *text0, const unsigned char *text1);

#ifdef __cplusplus
}
#endif

#endif /* STATEMACHINE_HELPERS_H_ */
