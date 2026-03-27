/************************************************************
 * @file    step.c
 * @brief   Implementation file for stepper motor control module
 * @author  dg
 * @date    4 Jan 2026
 ************************************************************/

/********************
 *     Includes		*
 ********************/
#include "step.h"
#include "log.h"
#include "math.h"
#include "peripherals.h"
#include "stdio.h"
#include "string.h"
#include "task_helpers.h"

/************************************
 *     Private Macros / Defines	   *
 ************************************/

#define STP_TIMER_FREQ_HZ 250000

/**
 * @brief Fixed-point scale factor for delay calculations
 * Using 16 fractional bits (scale by 65536) to maintain precision
 * This provides sufficient accuracy for high-speed applications
 * Based on principles from "Generate stepper-motor speed profiles in real time" by David Austin
 */
#define STP_DELAY_SCALE_BITS 16
#define STP_DELAY_SCALE (1 << STP_DELAY_SCALE_BITS) // 65536

/***************************
 *     Private Typedefs	   *
 ***************************/
typedef enum _STP_CmdType
{
    STP_CMD_MOVE,
    STP_CMD_STOP,
    STP_CMD_MOVE_PREPARE,
    STP_CMD_TRIGGER_START,
    STP_CMD_INIT_HANDLE,
    STP_CMD_SET_ACCELERATION,
    STP_CMD_SET_END_VELOCITY
} STP_CmdType_t;

typedef struct _STP_MoveCmdData
{
    int32_t steps; /**< Number of steps to execute (positive=CCW, negative=CW) */
} STP_MoveCmdData_t;

typedef struct _STP_StopCmdData
{
    uint8_t doDeceleration;
} STP_StopCmdData_t;

typedef struct _STP_InitHandleCmdData
{
    STP_StepperConfig_t config;
} STP_InitHandleCmdData_t;

typedef struct _STP_SetAccelerationCmdData
{
    double acceleration;
} STP_SetAccelerationCmdData_t;

typedef struct _STP_SetEndVelocityCmdData
{
    double endVelocity;
} STP_SetEndVelocityCmdData_t;

typedef struct _STP_CmdQueueItem
{
    STP_CmdType_t type;
    STP_Handle_t  handle;   /**< Pointer to target stepper handle */
    TickType_t    deadline; /**< Deadline for command completion (0 for no deadline) */
    THE_CmdHandle_t
        cmdHandle; /**< Command handle for task synchronization (NULL for no synchronization) */
    union
    {
        STP_MoveCmdData_t            move;
        STP_StopCmdData_t            stop;
        STP_InitHandleCmdData_t      config;
        STP_SetAccelerationCmdData_t setAcceleration;
        STP_SetEndVelocityCmdData_t  setEndVelocity;
    } data;
} STP_CmdQueueItem_t;

typedef struct _STP_AccelTablePoolItem
{
    uint16_t table[STP_MAX_ACCEL_TABLE_SIZE]; /**< Pre-calculated delay values */
    uint8_t  used;                            /**< Flag: 1 if in use, 0 if free */
    uint32_t tableSize;                       /**< Number of valid entries in table */
} STP_AccelTablePoolItem_t;

/**
 * @brief Structure for tracking stepper motor movement state and parameters.
 *
 * Contains all necessary information for executing a stepper motor movement,
 * including acceleration/deceleration profile, current step count, and motor state.
 */
typedef struct _STP_StepperMovement
{
    STP_MovementState_t state;            /**< Current movement state */
    uint32_t            totalSteps;       /**< Total steps to execute */
    uint32_t            accelSteps;       /**< Steps during acceleration phase */
    uint32_t            endVelocitySteps; /**< Steps at constant velocity */
    uint32_t            decelSteps;       /**< Steps during deceleration phase */
    STP_Direction_t     direction;        /**< Movement direction */
    uint8_t             isTrapezoidal;    /**< Flag: 1 if trapezoidal profile, 0 if triangular */
    uint32_t            currStepCount;    /**< Current step count during movement */
    uint32_t            phaseStepCount;   /**< Step count within current phase (for accel/decel) */
    uint16_t            endVelocityDelay; /**< Delay for constant velocity phase */
    int8_t   accelTablePoolIndex;         /**< Index into static pool (-1 if no table allocated) */
    uint32_t accelTableSize;              /**< Actual number of entries used in the table */
    uint16_t accelInterpFactor;           /**< Number of steps to repeat each table entry */
    uint16_t accelInterpCounter; /**< Counter for interpolation factor (0 to interpFactor-1) */
    uint32_t accelTableIndex;    /**< Current index into acceleration table */
    uint8_t  waitForStart; /**< Flag: 1 if movement is planned but awaiting sync start trigger */
    THE_CmdHandle_t
        cmdHandle; /**< Command handle for synchronizing movement start (NULL if no sync) */
} STP_StepperMovement_t;

/**
 * @brief Handle structure for a stepper motor instance.
 *
 * Contains hardware configuration and control parameters for a single stepper motor,
 * including FTM timer settings, GPIO pins, and movement parameters.
 */
typedef struct _STP_StepperHandleImpl
{
    FTM_Type*  ftmBase;            /**< Pointer to FTM module base address */
    ftm_chnl_t ftmChannel;         /**< FTM channel used for step output */
    PORT_Type* stepPort;           /**< PORT module for step pin mux control */
    GPIO_Type* stepGPIO;           /**< GPIO module for step pin */
    uint8_t    stepPin;            /**< Step output pin number */
    port_mux_t stepMuxFTM;         /**< Pin mux value for FTM mode */
    port_mux_t stepMuxGPIO;        /**< Pin mux value for GPIO mode */
    PORT_Type* dirPort;            /**< PORT module for direction pin mux control */
    GPIO_Type* dirGPIO;            /**< GPIO module for direction pin */
    uint8_t    dirPin;             /**< Direction output pin number */
    port_mux_t dirMux;             /**< Pin mux value for dir pin */
    uint8_t dirLogicHighClockwise; /**< Flag: 1 if high = clockwise, 0 if high = counterclockwise */
    int32_t absolutePosition;      /**< The absolute Position of the StepperMotor in steps. Positive
                                      Values are counterclockwise*/
    double                acceleration;   /**< Acceleration in steps/s² */
    double                endVelocity;    /**< Final velocity in steps/s */
    const char*           label;          /**< Identifier label for logging (e.g., "Motor_X") */
    STP_StepperMovement_t movementHandle; /**< Current movement state and parameters */
} STP_StepperHandleImpl_t;

typedef struct _STP_HandlesArrayItem
{
    STP_StepperHandleImpl_t handle; /**< Stepper motor handle */
    uint8_t                 used;   /**< Flag: 1 if handle is allocated, 0 if free */
} STP_HandlesArrayItem_t;

/*****************************************
 *     Private Function Declarations	   *
 *****************************************/

static status_t init_handle(const STP_StepperConfig_t config, TickType_t deadline);
static status_t step_pin_mux_gpio(STP_Handle_t handle);
static status_t step_pin_mux_ftm(STP_Handle_t handle);
static status_t plan_motion_profile(STP_Handle_t handle);
static status_t calculate_acceleration_profile(uint16_t* lookupTable, uint32_t numSteps,
                                               uint16_t stepZeroDelay, uint16_t endVelocityDelay);
static status_t allocate_accel_table(uint32_t tableSize, int8_t* poolIndex);
static void     free_accel_table(int8_t poolIndex);
static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_Handle_t* handle);
static status_t    reset_movement_handle(STP_StepperMovement_t* handle);
static void        process(void);
static status_t    init_dir_pin(STP_Handle_t handle);
static status_t    set_direction_pin(STP_Handle_t handle);
static status_t    init_step_pin(STP_Handle_t handle);
static inline void calculate_step_profile(STP_Handle_t handle);
static inline void calculate_delays(STP_Handle_t handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay);
static inline void calculate_acceleration_table_parameters(STP_Handle_t handle,
                                                           uint32_t*    interpFactor,
                                                           uint32_t*    tableSize);
static status_t create_accel_table(STP_Handle_t handle, uint32_t tableSize, uint16_t stepZeroDelay,
                                   uint16_t endVelocityDelay, int8_t* poolIndex);
static inline void update_isr_handle_cache(void);

static status_t reset_absolute_position(STP_Handle_t handle);
static void     check_movement_completion(void);
static void     process_cmd(STP_CmdQueueItem_t queueItem);
static status_t start_steps(STP_Handle_t handle, THE_CmdHandle_t cmdHandle);
static status_t send_cmd_async(STP_CmdQueueItem_t* queueItem, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle);

/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

static STP_HandlesArrayItem_t handles[STP_MAX_HANDLE_COUNT];

static THE_CmdHandleImpl_t cmdHandles[STP_MAX_CMD_HANDLE_COUNT];

static QueueHandle_t cmdQueue = NULL;

STP_Handle_t FTM3_ISR_handle_cache[8] = {NULL};

static uint8_t    isrDebugPin  = 1;
static GPIO_Type* isrDebugGPIO = GPIOA;
static PORT_Type* isrDebugPort = PORTA;

__attribute__((section(
    ".accelTables"))) static STP_AccelTablePoolItem_t accelTablePool[STP_ACCEL_TABLE_POOL_SIZE];

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

status_t STP_init()
{
    // Initialize acceleration table pool
    for (int8_t i = 0; i < STP_ACCEL_TABLE_POOL_SIZE; i++)
    {
        accelTablePool[i].used      = 0;
        accelTablePool[i].tableSize = 0;
    }

    cmdQueue = xQueueCreate(STP_CMD_QUEUE_SIZE, sizeof(STP_CmdQueueItem_t));
    if (cmdQueue == NULL)
    {
        return kStatus_Fail;
    }

    SIM->SCGC6 |= SIM_SCGC6_FTM3_MASK;

    NVIC_SetPriority(FTM3_IRQn, 8);
    NVIC_EnableIRQ(FTM3_IRQn);

    FTM3->MOD = 0xFFFF;
    FTM3->SC  = FTM_SC_CLKS(2) | FTM_SC_PS(0);

    gpio_pin_config_t config;
    config.pinDirection = kGPIO_DigitalOutput;

    GPIO_PinInit(isrDebugGPIO, isrDebugPin, &config);

    PORT_SetPinMux(isrDebugPort, isrDebugPin, kPORT_MuxAlt1);
    GPIO_PinWrite(isrDebugGPIO, isrDebugPin, 0);

    THE_init_cmd_handles(cmdHandles, STP_MAX_CMD_HANDLE_COUNT);

    return kStatus_Success;
}

void STP_task(void* pvParameters)
{
    LOG_INFO("Started Step Task");
    for (;;)
    {
        process();
    }
}

status_t STP_stop_async(STP_Handle_t handle, uint8_t doDeceleration, TickType_t deadline,
                        THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem       = {0};
    queueItem.handle                   = handle;
    queueItem.data.stop.doDeceleration = doDeceleration;
    queueItem.type                     = STP_CMD_STOP;
    queueItem.deadline                 = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_move_relative_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                 THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.handle             = handle;
    queueItem.data.move.steps    = steps;
    queueItem.type               = STP_CMD_MOVE;
    queueItem.deadline           = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_init_handle_async(STP_StepperConfig_t config, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.data.config.config = config;
    queueItem.type               = STP_CMD_INIT_HANDLE;
    queueItem.deadline           = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_handle_by_label(const char* label, STP_Handle_t* handle)
{
    if (label == NULL || handle == NULL)
    {
        return kStatus_Fail;
    }
    for (uint8_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].used)
        {
            uint32_t handleLength = strlen(handles[i].handle.label);

            if (strncmp(label, handles[i].handle.label, handleLength) == 0)
            {
                *handle = &handles[i].handle;
                return kStatus_Success;
            }
        }
    }
    return kStatus_Fail;
}

status_t STP_reset_absolute_position(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    return reset_absolute_position(handle);
}

status_t STP_move_relative_prepare_async(STP_Handle_t handle, int32_t steps, TickType_t deadline,
                                         THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.handle             = handle;
    queueItem.type               = STP_CMD_MOVE_PREPARE;
    queueItem.data.move.steps    = steps;
    queueItem.deadline           = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_trigger_prepared_moves_async(TickType_t deadline, THE_CmdHandle_t* cmdHandle)
{
    if (cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem = {0};
    queueItem.type               = STP_CMD_TRIGGER_START;
    queueItem.deadline           = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_absolute_steps(STP_Handle_t handle, int32_t* absoluteSteps)
{
    if (handle == NULL || absoluteSteps == NULL)
    {
        return kStatus_Fail;
    }
    *absoluteSteps = handle->absolutePosition;
    return kStatus_Success;
}

status_t STP_get_acceleration(STP_Handle_t handle, double* acceleration)
{
    if (handle == NULL || acceleration == NULL)
    {
        return kStatus_Fail;
    }
    *acceleration = handle->acceleration;
    return kStatus_Success;
}

status_t STP_get_end_velocity(STP_Handle_t handle, double* endVelocity)
{
    if (handle == NULL || endVelocity == NULL)
    {
        return kStatus_Fail;
    }
    *endVelocity = handle->endVelocity;
    return kStatus_Success;
}

status_t STP_get_movement_state(STP_Handle_t handle, STP_MovementState_t* state)
{
    if (handle == NULL || state == NULL)
    {
        return kStatus_Fail;
    }
    *state = handle->movementHandle.state;
    return kStatus_Success;
}

status_t STP_set_acceleration_async(STP_Handle_t handle, double acceleration, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem                = {0};
    queueItem.handle                            = handle;
    queueItem.type                              = STP_CMD_SET_ACCELERATION;
    queueItem.data.setAcceleration.acceleration = acceleration;
    queueItem.deadline                          = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_set_end_velocity_async(STP_Handle_t handle, double endVelocity, TickType_t deadline,
                                    THE_CmdHandle_t* cmdHandle)
{
    if (handle == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    if (endVelocity <= 0.0 || endVelocity > STP_MAX_STEP_FREQUENCY)
    {
        return kStatus_Fail;
    }

    STP_CmdQueueItem_t queueItem              = {0};
    queueItem.handle                          = handle;
    queueItem.type                            = STP_CMD_SET_END_VELOCITY;
    queueItem.data.setEndVelocity.endVelocity = endVelocity;
    queueItem.deadline                        = deadline;

    return send_cmd_async(&queueItem, deadline, cmdHandle);
}

status_t STP_get_default_config(STP_StepperConfig_t* config)
{
    if (config == NULL)
    {
        return kStatus_Fail;
    }

    config->acceleration          = 200;
    config->endVelocity           = 360;
    config->dirGPIO               = GPIOA;
    config->dirPin                = 1;
    config->dirPort               = PORTA;
    config->ftmBase               = FTM0;
    config->ftmChannel            = kFTM_Chnl_0;
    config->stepGPIO              = GPIOA;
    config->stepPin               = 0;
    config->stepPort              = PORTA;
    config->stepMuxFTM            = kPORT_MuxAlt4;
    config->stepMuxGPIO           = kPORT_MuxAlt1;
    config->dirMux                = kPORT_MuxAlt1;
    config->label                 = "stepperMotor";
    config->dirLogicHighClockwise = 1;
    return kStatus_Success;
}

/********************************************
 *     Private Function Implementations	   *
 ********************************************/

static status_t send_cmd_async(STP_CmdQueueItem_t* queueItem, TickType_t deadline,
                               THE_CmdHandle_t* cmdHandle)
{
    if (queueItem == NULL || cmdHandle == NULL)
    {
        return kStatus_Fail;
    }

    if (THE_get_cmd_handle(cmdHandle, cmdHandles, STP_MAX_CMD_HANDLE_COUNT) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    queueItem->cmdHandle = *cmdHandle;

    if (THE_send_cmd(cmdQueue, (void*)queueItem, deadline, *cmdHandle) != kStatus_Success)
    {
        THE_remove_cmd_handle_ref(*cmdHandle);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

// TODO check for correct step count
void FTM3_IRQHandler(void)
{
    uint32_t intStatus;
    /* Reading all interrupt flags of status register */
    intStatus = FTM_GetStatusFlags(FTM3);
    FTM_ClearStatusFlags(FTM3, intStatus);
    GPIO_PinWrite(isrDebugGPIO, isrDebugPin, 1);

    /* Place your code here */
    for (uint8_t i = 0; i < 8; i++)
    {
        STP_Handle_t stepperHandle;

        if (intStatus & (0b1 << i))
        {
            if (FTM3_ISR_handle_cache[i] == NULL)
            {
                continue;
            }
            stepperHandle = FTM3_ISR_handle_cache[i];

            stepperHandle->movementHandle.currStepCount++;
            stepperHandle->absolutePosition +=
                (stepperHandle->movementHandle.direction == STP_COUNTERCLOCKWISE) ? 1 : -1;
            uint32_t currentStepCount   = stepperHandle->movementHandle.currStepCount;
            uint32_t totalSteps         = stepperHandle->movementHandle.totalSteps;
            uint32_t accelSteps         = stepperHandle->movementHandle.accelSteps;
            uint32_t constVelocitySteps = stepperHandle->movementHandle.endVelocitySteps;

            if (currentStepCount >= totalSteps)
            {
                stepperHandle->ftmBase->CONTROLS[stepperHandle->ftmChannel].CnSC &=
                    ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
                stepperHandle->movementHandle.state = STP_MOVEMENT_FINISHED;
                continue;
            }
            if (stepperHandle->movementHandle.state == STP_MOVEMENT_STARTED)
            {
                stepperHandle->movementHandle.state              = STP_MOVEMENT_ACCELERATING;
                stepperHandle->movementHandle.phaseStepCount     = 0;
                stepperHandle->movementHandle.accelInterpCounter = 0;
                stepperHandle->movementHandle.accelTableIndex    = 0;
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_ACCELERATING &&
                     currentStepCount > stepperHandle->movementHandle.accelSteps)
            {
                if (stepperHandle->movementHandle.isTrapezoidal)
                {
                    stepperHandle->movementHandle.state = STP_MOVEMENT_CONST_VELOCITY;
                }
                else
                {
                    stepperHandle->movementHandle.state              = STP_MOVEMENT_DECELRATING;
                    stepperHandle->movementHandle.phaseStepCount     = 0;
                    stepperHandle->movementHandle.accelInterpCounter = 0;
                    stepperHandle->movementHandle.accelTableIndex =
                        stepperHandle->movementHandle.accelTableSize - 1;
                }
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_CONST_VELOCITY &&
                     currentStepCount > accelSteps + constVelocitySteps)
            {
                stepperHandle->movementHandle.state              = STP_MOVEMENT_DECELRATING;
                stepperHandle->movementHandle.phaseStepCount     = 0;
                stepperHandle->movementHandle.accelInterpCounter = 0;
                stepperHandle->movementHandle.accelTableIndex =
                    stepperHandle->movementHandle.accelTableSize - 1;
            }

            // Increment phase step counter
            stepperHandle->movementHandle.phaseStepCount++;

            uint16_t actualDelay;
            int8_t   poolIndex = stepperHandle->movementHandle.accelTablePoolIndex;

            if (stepperHandle->movementHandle.state == STP_MOVEMENT_ACCELERATING)
            {
                // Use counter-based repeat factor (no division!)
                if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
                    stepperHandle->movementHandle.accelTableIndex <
                        accelTablePool[poolIndex].tableSize)
                {
                    actualDelay = accelTablePool[poolIndex]
                                      .table[stepperHandle->movementHandle.accelTableIndex];
                    // Increment counter and advance table index when counter reaches factor
                    stepperHandle->movementHandle.accelInterpCounter++;
                    if (stepperHandle->movementHandle.accelInterpCounter >=
                        stepperHandle->movementHandle.accelInterpFactor)
                    {
                        stepperHandle->movementHandle.accelInterpCounter = 0;
                        stepperHandle->movementHandle.accelTableIndex++;
                    }
                }
                else
                {
                    actualDelay = stepperHandle->movementHandle.endVelocityDelay;
                }
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_CONST_VELOCITY)
            {
                // Constant velocity: use endVelocityDelay directly
                actualDelay = stepperHandle->movementHandle.endVelocityDelay;
            }
            else if (stepperHandle->movementHandle.state == STP_MOVEMENT_DECELRATING)
            {
                // Use counter-based repeat factor reading table backwards (no division!)
                if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
                    stepperHandle->movementHandle.accelTableIndex <
                        accelTablePool[poolIndex].tableSize)
                {
                    actualDelay = accelTablePool[poolIndex]
                                      .table[stepperHandle->movementHandle.accelTableIndex];
                    // Increment counter and decrement table index when counter reaches factor
                    stepperHandle->movementHandle.accelInterpCounter++;
                    if (stepperHandle->movementHandle.accelInterpCounter >=
                        stepperHandle->movementHandle.accelInterpFactor)
                    {
                        stepperHandle->movementHandle.accelInterpCounter = 0;
                        if (stepperHandle->movementHandle.accelTableIndex > 0)
                        {
                            stepperHandle->movementHandle.accelTableIndex--;
                        }
                    }
                }
                else
                {
                    actualDelay = stepperHandle->movementHandle.endVelocityDelay;
                }
            }
            else
            {
                actualDelay = stepperHandle->movementHandle.endVelocityDelay;
            }
            stepperHandle->ftmBase->CONTROLS[stepperHandle->ftmChannel].CnV += actualDelay;
        }
    }

    GPIO_PinWrite(isrDebugGPIO, isrDebugPin, 0);

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
   Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static status_t init_handle(const STP_StepperConfig_t config, TickType_t deadline)
{
    static char logMsg[90];

    STP_Handle_t handle = NULL;
    for (uint8_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].used == 0)
        {
            handles[i].used = 1;
            handle          = &handles[i].handle;
            break;
        }
    }

    if (handle == NULL)
    {
        LOG_ERROR("Failed to init stepper handle. Too many handles.");

        return kStatus_Fail;
    }

    handle->ftmBase    = config.ftmBase;
    handle->ftmChannel = config.ftmChannel;

    handle->stepPort    = config.stepPort;
    handle->stepGPIO    = config.stepGPIO;
    handle->stepPin     = config.stepPin;
    handle->stepMuxFTM  = config.stepMuxFTM;
    handle->stepMuxGPIO = config.stepMuxGPIO;

    handle->dirPort               = config.dirPort;
    handle->dirGPIO               = config.dirGPIO;
    handle->dirPin                = config.dirPin;
    handle->dirLogicHighClockwise = config.dirLogicHighClockwise;

    handle->acceleration     = config.acceleration;
    handle->endVelocity      = config.endVelocity;
    handle->label            = config.label;
    handle->dirMux           = config.dirMux;
    handle->absolutePosition = 0;

    if (reset_movement_handle(&handle->movementHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (init_step_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (init_dir_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    update_isr_handle_cache();

    snprintf(logMsg, sizeof(logMsg), "[%s] stepper handle initialized: accel=%.1f, endVel=%.1f",
             handle->label, handle->acceleration, handle->endVelocity);
    LOG_INFO(logMsg);

    return kStatus_Success;
}

static status_t reset_absolute_position(STP_Handle_t handle)
{
    taskENTER_CRITICAL();
    handle->absolutePosition = 0;
    taskEXIT_CRITICAL();
    return kStatus_Success;
}

static status_t init_step_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    gpio_pin_config_t gpioConfig;
    gpioConfig.pinDirection = kGPIO_DigitalOutput;
    gpioConfig.outputLogic  = GPIO_PinRead(handle->stepGPIO, handle->stepPin);

    GPIO_PinInit(handle->stepGPIO, handle->stepPin, &gpioConfig);

    return kStatus_Success;
}

static status_t init_dir_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    gpio_pin_config_t config;
    config.pinDirection = kGPIO_DigitalOutput;

    GPIO_PinInit(handle->dirGPIO, handle->dirPin, &config);

    PORT_SetPinMux(handle->dirPort, handle->dirPin, handle->dirMux);
    GPIO_PinWrite(handle->dirGPIO, handle->dirPin, 1);

    return kStatus_Success;
}

static status_t step_pin_mux_gpio(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    gpio_pin_config_t config;
    config.pinDirection = kGPIO_DigitalOutput;
    config.outputLogic  = GPIO_PinRead(handle->stepGPIO, handle->stepPin);

    GPIO_PinInit(handle->stepGPIO, handle->stepPin, &config);

    PORT_SetPinMux(handle->stepPort, handle->stepPin, handle->stepMuxGPIO);
    return kStatus_Success;
}

static status_t step_pin_mux_ftm(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    gpio_pin_config_t config;
    config.pinDirection = kGPIO_DigitalInput;

    GPIO_PinInit(handle->stepGPIO, handle->stepPin, &config);

    PORT_SetPinMux(handle->stepPort, handle->stepPin, handle->stepMuxFTM);
    return kStatus_Success;
}

static status_t plan_motion_profile(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    calculate_step_profile(handle);

    uint16_t stepZeroDelay;
    uint16_t endVelocityDelay;
    calculate_delays(handle, &stepZeroDelay, &endVelocityDelay);

    uint32_t interpFactor;
    uint32_t tableSize;
    calculate_acceleration_table_parameters(handle, &interpFactor, &tableSize);

    int8_t poolIndex;
    if (create_accel_table(handle, tableSize, stepZeroDelay, endVelocityDelay, &poolIndex) !=
        kStatus_Success)
    {
        return kStatus_Fail;
    }

    // Store table information in movement handle
    handle->movementHandle.accelTablePoolIndex = poolIndex;
    handle->movementHandle.accelTableSize      = tableSize;
    handle->movementHandle.accelInterpFactor   = (uint16_t)interpFactor;

    handle->movementHandle.currStepCount = 0;
    handle->movementHandle.state         = STP_MOVEMENT_PLANNED;

    static char logMsg[100];
    if (handle->movementHandle.isTrapezoidal)
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Motion planned: trapezoidal (accel:%lu const:%lu decel:%lu)", handle->label,
                 handle->movementHandle.accelSteps, handle->movementHandle.endVelocitySteps,
                 handle->movementHandle.decelSteps);
    }
    else
    {
        snprintf(logMsg, sizeof(logMsg), "[%s] Motion planned: triangular (accel:%lu decel:%lu)",
                 handle->label, handle->movementHandle.accelSteps,
                 handle->movementHandle.decelSteps);
    }
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

static inline void calculate_step_profile(STP_Handle_t handle)
{
    uint32_t accelSteps =
        (uint32_t)((handle->endVelocity * handle->endVelocity) / (2 * handle->acceleration));

    if (accelSteps * 2 >= handle->movementHandle.totalSteps)
    {

        handle->movementHandle.isTrapezoidal = 0;

        handle->movementHandle.accelSteps = handle->movementHandle.totalSteps / 2;

        handle->movementHandle.decelSteps =
            handle->movementHandle.totalSteps - handle->movementHandle.accelSteps;

        handle->movementHandle.endVelocitySteps = 0;
    }
    else
    {

        handle->movementHandle.isTrapezoidal = 1;

        handle->movementHandle.accelSteps = accelSteps;
        handle->movementHandle.decelSteps = accelSteps;

        handle->movementHandle.endVelocitySteps =
            handle->movementHandle.totalSteps - 2 * accelSteps;
    }
}

static inline void calculate_delays(STP_Handle_t handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay)
{
    *stepZeroDelay = (uint16_t)(0.676 * STP_TIMER_FREQ_HZ * sqrt(2.0 / (handle->acceleration)));

    *endVelocityDelay = (uint16_t)(STP_TIMER_FREQ_HZ / (handle->endVelocity));

    handle->movementHandle.endVelocityDelay = *endVelocityDelay;
}

static inline void calculate_acceleration_table_parameters(STP_Handle_t handle,
                                                           uint32_t*    interpFactor,
                                                           uint32_t*    tableSize)
{
    *interpFactor = STP_ACCEL_TABLE_INTERP_FACTOR;

    if (*interpFactor < 1)
    {
        *interpFactor = 1;
    }

    if (handle->movementHandle.accelSteps > STP_MAX_ACCEL_TABLE_SIZE * (*interpFactor))
    {
        *interpFactor = (handle->movementHandle.accelSteps + STP_MAX_ACCEL_TABLE_SIZE - 1) /
                        STP_MAX_ACCEL_TABLE_SIZE;
    }

    *tableSize = (handle->movementHandle.accelSteps + *interpFactor - 1) / *interpFactor;

    if (*tableSize > STP_MAX_ACCEL_TABLE_SIZE)
    {
        *tableSize = STP_MAX_ACCEL_TABLE_SIZE;
    }
}

static status_t create_accel_table(STP_Handle_t handle, uint32_t tableSize, uint16_t stepZeroDelay,
                                   uint16_t endVelocityDelay, int8_t* poolIndex)
{

    if (handle == NULL || poolIndex == NULL)
    {
        return kStatus_Fail;
    }

    status_t status;

    status = allocate_accel_table(tableSize, poolIndex);

    if (status != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (tableSize > 0)
    {
        status = calculate_acceleration_profile(accelTablePool[*poolIndex].table, tableSize,
                                                stepZeroDelay, endVelocityDelay);

        if (status != kStatus_Success)
        {
            free_accel_table(*poolIndex);
            return kStatus_Fail;
        }
    }

    return kStatus_Success;
}

static inline void update_isr_handle_cache(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        STP_Handle_t stepperHandle;
        get_handle_from_timer(FTM3, i, &stepperHandle);
        FTM3_ISR_handle_cache[i] = stepperHandle;
    }
}

static status_t allocate_accel_table(uint32_t tableSize, int8_t* poolIndex)
{
    if (poolIndex == NULL)
    {
        return kStatus_Fail;
    }

    for (int8_t i = 0; i < STP_ACCEL_TABLE_POOL_SIZE; i++)
    {
        if (accelTablePool[i].used == 0)
        {
            accelTablePool[i].used      = 1;
            accelTablePool[i].tableSize = tableSize;
            *poolIndex                  = i;
            return kStatus_Success;
        }
    }
    *poolIndex = -1;
    return kStatus_Fail; // No free table available
}

static void free_accel_table(int8_t poolIndex)
{
    if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE)
    {
        accelTablePool[poolIndex].used      = 0;
        accelTablePool[poolIndex].tableSize = 0;
    }
}

static status_t calculate_acceleration_profile(uint16_t* lookupTable, uint32_t numSteps,
                                               uint16_t stepZeroDelay, uint16_t endVelocityDelay)
{
    if (lookupTable == NULL || numSteps == 0)
    {
        return kStatus_Fail;
    }

    // Start with initial delay, scaled up for fixed-point arithmetic
    uint32_t currentDelay = ((uint32_t)stepZeroDelay) << STP_DELAY_SCALE_BITS;
    uint32_t minDelay     = ((uint32_t)endVelocityDelay) << STP_DELAY_SCALE_BITS;

    // Calculate each step delay using David Austin formula
    for (uint32_t n = 1; n <= numSteps; n++)
    {
        // Store current delay (scaled down to uint16_t)
        lookupTable[n - 1] = (uint16_t)(currentDelay >> STP_DELAY_SCALE_BITS);

        // Calculate next delay: c_n = c_(n-1) - 2*c_(n-1) / (4*n + 1)
        // Using fixed-point arithmetic to maintain fractional precision
        uint32_t nextDelay = currentDelay - (2 * currentDelay / (4 * n + 1));

        // Clamp to minimum delay (end velocity)
        if (nextDelay < minDelay)
        {
            nextDelay = minDelay;
        }

        currentDelay = nextDelay;
    }
    return kStatus_Success;
}

static status_t start_steps(STP_Handle_t handle, THE_CmdHandle_t cmdHandle)
{
    static char logMsg[100];

    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    if (handle->movementHandle.state != STP_MOVEMENT_PLANNED)
    {
        LOG_ERROR("Cannot start movement that is not in PLANNED state");
        return kStatus_Fail;
    }

    if (step_pin_mux_ftm(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (set_direction_pin(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~FTM_CnSC_CHF_MASK;

    // Use first value from lookup table
    uint16_t initialDelay;
    int8_t   poolIndex = handle->movementHandle.accelTablePoolIndex;

    if (poolIndex >= 0 && poolIndex < STP_ACCEL_TABLE_POOL_SIZE &&
        accelTablePool[poolIndex].tableSize > 0)
    {
        initialDelay = accelTablePool[poolIndex].table[0];
    }
    else
    {
        snprintf(logMsg, sizeof(logMsg),
                 "[%s] Failed to retrieve initial delay from lookup table for %lu steps",
                 handle->label, handle->movementHandle.totalSteps);
        LOG_ERROR(logMsg);
        return kStatus_Fail;
    }
    handle->ftmBase->CONTROLS[handle->ftmChannel].CnV = handle->ftmBase->CNT + initialDelay;
    handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC |=
        FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK;

    handle->movementHandle.state     = STP_MOVEMENT_STARTED;
    handle->movementHandle.cmdHandle = cmdHandle;

    snprintf(logMsg, sizeof(logMsg), "[%s] Movement started: %lu steps", handle->label,
             handle->movementHandle.totalSteps);
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_Handle_t* handle)
{
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].handle.ftmBase == ftmBase && handles[i].handle.ftmChannel == ftmChannel &&
            handles[i].used == 1)
        {
            *handle = &handles[i].handle;
            return;
        }
    }
    *handle = NULL;
    return;
}

static status_t reset_movement_handle(STP_StepperMovement_t* handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    // Free lookup table back to pool if allocated
    if (handle->accelTablePoolIndex >= 0)
    {
        free_accel_table(handle->accelTablePoolIndex);
        handle->accelTablePoolIndex = -1;
    }

    handle->state              = STP_MOVEMENT_IDLE;
    handle->totalSteps         = 0;
    handle->accelSteps         = 0;
    handle->endVelocitySteps   = 0;
    handle->decelSteps         = 0;
    handle->direction          = STP_CLOCKWISE;
    handle->isTrapezoidal      = 0;
    handle->currStepCount      = 0;
    handle->phaseStepCount     = 0;
    handle->endVelocityDelay   = 0;
    handle->accelTableSize     = 0;
    handle->accelInterpFactor  = 1;
    handle->accelInterpCounter = 0;
    handle->accelTableIndex    = 0;
    handle->waitForStart       = 0;
    handle->cmdHandle          = NULL;
    return kStatus_Success;
}

static status_t stop_steps(STP_Handle_t handle, uint8_t doDeceleration, THE_CmdHandle_t cmdHandle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    THE_CmdHandle_t oldCmdHandle = NULL;

    taskENTER_CRITICAL();

    STP_MovementState_t st = handle->movementHandle.state;
    if (st == STP_MOVEMENT_IDLE || st == STP_MOVEMENT_FINISHED || st == STP_MOVEMENT_STOPPED)
    {
        taskEXIT_CRITICAL();
        return kStatus_Fail;
    }

    oldCmdHandle = handle->movementHandle.cmdHandle;

    if (doDeceleration == 0)
    {
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &=
            ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~(FTM_CnSC_CHF_MASK);
        handle->movementHandle.state = STP_MOVEMENT_STOPPED;
    }
    else
    {
        if (st == STP_MOVEMENT_PLANNED)
        {
            // nothing running yet -> stop immediately
            handle->movementHandle.state = STP_MOVEMENT_STOPPED;
        }
        else if (st == STP_MOVEMENT_ACCELERATING)
        {
            handle->movementHandle.isTrapezoidal    = 0;
            handle->movementHandle.endVelocitySteps = 0;
            handle->movementHandle.accelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.decelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.totalSteps       = handle->movementHandle.currStepCount * 2;
        }
        else if (st == STP_MOVEMENT_CONST_VELOCITY)
        {
            handle->movementHandle.endVelocitySteps =
                handle->movementHandle.currStepCount - handle->movementHandle.accelSteps;
            handle->movementHandle.totalSteps = handle->movementHandle.accelSteps +
                                                handle->movementHandle.endVelocitySteps +
                                                handle->movementHandle.decelSteps;
        }
        // if already decelerating: keep as-is
    }

    handle->movementHandle.cmdHandle = cmdHandle;
    taskEXIT_CRITICAL();

    if (oldCmdHandle != NULL && oldCmdHandle != cmdHandle)
    {
        THE_notify_task_failure(oldCmdHandle);
        THE_remove_cmd_handle_ref(oldCmdHandle);
    }

    return kStatus_Success;
}

static status_t set_direction_pin(STP_Handle_t handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint8_t pinValue;
    if (handle->dirLogicHighClockwise)
    {
        pinValue = handle->movementHandle.direction == STP_CLOCKWISE;
    }
    else
    {
        pinValue = handle->movementHandle.direction == STP_COUNTERCLOCKWISE;
    }
    GPIO_PinWrite(handle->dirGPIO, handle->dirPin, pinValue);

    return kStatus_Success;
}

static void check_movement_completion(void)
{
    static char logMsg[80];

    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].used == 1 &&
            (handles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED ||
             handles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED))
        {

            step_pin_mux_gpio(&handles[i].handle);

            if (handles[i].handle.movementHandle.cmdHandle != NULL)
            {
                THE_notify_task_success(handles[i].handle.movementHandle.cmdHandle);
                THE_remove_cmd_handle_ref(handles[i].handle.movementHandle.cmdHandle);
            }

            if (handles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED)
            {
                snprintf(logMsg, sizeof(logMsg), "[%s] Movement finished", handles[i].handle.label);
            }
            else if (handles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED)
            {
                snprintf(logMsg, sizeof(logMsg), "[%s] Movement stopped", handles[i].handle.label);
            }
            LOG_DEBUG(logMsg);

            reset_movement_handle(&handles[i].handle.movementHandle);
        }
    }
}

static void process_cmd(STP_CmdQueueItem_t queueItem)
{
    switch (queueItem.type)
    {
        case STP_CMD_MOVE:
        {
            LOG_DEBUG("Processing movement command");
            int32_t steps                               = queueItem.data.move.steps;
            queueItem.handle->movementHandle.totalSteps = (steps >= 0) ? steps : -steps;
            queueItem.handle->movementHandle.direction =
                (steps >= 0) ? STP_COUNTERCLOCKWISE : STP_CLOCKWISE;
            queueItem.handle->movementHandle.waitForStart = 0;
            if (plan_motion_profile(queueItem.handle) == kStatus_Success)
            {
                if (start_steps(queueItem.handle, queueItem.cmdHandle) != kStatus_Success)
                {
                    THE_notify_task_failure(queueItem.cmdHandle);
                    THE_remove_cmd_handle_ref(queueItem.cmdHandle);
                    LOG_ERROR("Failed to start steps");
                }
            }
            else
            {
                THE_notify_task_failure(queueItem.cmdHandle);
                THE_remove_cmd_handle_ref(queueItem.cmdHandle);
                LOG_ERROR("Failed to plan motion profile");
            }

            break;
        }
        case STP_CMD_MOVE_PREPARE:
        {
            LOG_DEBUG("Processing prepare movement command");
            int32_t steps                               = queueItem.data.move.steps;
            queueItem.handle->movementHandle.totalSteps = (steps >= 0) ? steps : -steps;
            queueItem.handle->movementHandle.direction =
                (steps >= 0) ? STP_COUNTERCLOCKWISE : STP_CLOCKWISE;
            queueItem.handle->movementHandle.waitForStart = 1;
            if (plan_motion_profile(queueItem.handle) == kStatus_Success)
            {
                THE_notify_task_success(queueItem.cmdHandle);
            }
            else
            {
                THE_notify_task_failure(queueItem.cmdHandle);
            }
            THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_TRIGGER_START:
        {
            LOG_DEBUG("Triggering synchronized start");

            uint8_t anyStarted = 0;
            uint8_t ownerSet   = 0;

            for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
            {
                if (handles[i].used == 1 &&
                    handles[i].handle.movementHandle.state == STP_MOVEMENT_PLANNED &&
                    handles[i].handle.movementHandle.waitForStart == 1)
                {
                    handles[i].handle.movementHandle.waitForStart = 0;

                    THE_CmdHandle_t h = ownerSet ? NULL : queueItem.cmdHandle;
                    if (start_steps(&handles[i].handle, h) == kStatus_Success)
                    {
                        anyStarted = 1;
                        if (!ownerSet)
                        {
                            ownerSet =
                                1; // only first successfully started motor owns trigger handle
                        }
                    }
                }
            }

            if (!anyStarted)
            {
                THE_notify_task_failure(queueItem.cmdHandle);
                THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            }
            break;
        }
        case STP_CMD_STOP:
        {
            LOG_DEBUG("Processing stop command");
            if (stop_steps(queueItem.handle, queueItem.data.stop.doDeceleration,
                           queueItem.cmdHandle) != kStatus_Success)
            {
                THE_notify_task_failure(queueItem.cmdHandle);
                THE_remove_cmd_handle_ref(queueItem.cmdHandle);
                LOG_ERROR("Failed to stop steps");
            }
            break;
        }
        case STP_CMD_INIT_HANDLE:
        {
            LOG_DEBUG("Processing init handle command");
            if (init_handle(queueItem.data.config.config, queueItem.deadline) == kStatus_Success)
            {
                THE_notify_task_success(queueItem.cmdHandle);
            }
            else
            {
                THE_notify_task_failure(queueItem.cmdHandle);
            }
            THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_SET_ACCELERATION:
        {
            LOG_DEBUG("Processing set acceleration command");
            queueItem.handle->acceleration = queueItem.data.setAcceleration.acceleration;
            THE_notify_task_success(queueItem.cmdHandle);
            THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        case STP_CMD_SET_END_VELOCITY:
        {
            LOG_DEBUG("Processing set end velocity command");
            queueItem.handle->endVelocity = queueItem.data.setEndVelocity.endVelocity;
            THE_notify_task_success(queueItem.cmdHandle);
            THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            break;
        }
        default:
        {
            THE_notify_task_failure(queueItem.cmdHandle);
            THE_remove_cmd_handle_ref(queueItem.cmdHandle);
            LOG_WARN("Received unknown command type");
            break;
        }
    }
}

static void process(void)
{
    check_movement_completion();

    STP_CmdQueueItem_t queueItem = {0};
    BaseType_t         status;
    status = xQueueReceive(cmdQueue, &queueItem, pdMS_TO_TICKS(1));
    if (status != pdPASS)
    {
        return;
    }
    process_cmd(queueItem);
}
