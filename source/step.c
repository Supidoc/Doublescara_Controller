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
    STP_CMD_STOP
} STP_CmdType_t;

typedef struct _STP_MoveCmdData
{
    uint32_t        stepCount; /**< Number of steps to execute */
    STP_Direction_t direction; /**< Direction of movement */
} STP_MoveCmdData_t;

typedef struct _STP_StopCmdData
{
    uint8_t doDeceleration;
} STP_StopCmdData_t;

typedef struct _STP_CmdQueueItem
{
    STP_CmdType_t        type;
    STP_StepperHandle_t* handle; /**< Pointer to target stepper handle */
    union
    {
        STP_MoveCmdData_t move;
        STP_StopCmdData_t stop;
    } data;
} STP_CmdQueueItem_t;

typedef struct _STP_HandlesArrayItem
{
    STP_StepperHandle_t handle; /**< Stepper motor handle */
    uint8_t             used;   /**< Flag: 1 if handle is allocated, 0 if free */
} STP_HandlesArrayItem_t;

/**
 * @brief Pool item for static acceleration lookup tables.
 */
typedef struct _STP_AccelTablePoolItem
{
    uint16_t table[STP_MAX_ACCEL_TABLE_SIZE]; /**< Pre-calculated delay values */
    uint8_t  used;                            /**< Flag: 1 if in use, 0 if free */
    uint32_t tableSize;                       /**< Number of valid entries in table */
} STP_AccelTablePoolItem_t;

/*****************************************
 *     Private Function Declarations	   *
 *****************************************/

static status_t    init_handle(const STP_StepperConfig_t* config);
static status_t    step_pin_mux_gpio(STP_StepperHandle_t* handle);
static status_t    step_pin_mux_ftm(STP_StepperHandle_t* handle);
static status_t    plan_motion_profile(STP_StepperHandle_t* handle);
static status_t    calculate_acceleration_profile(uint16_t* lookupTable, uint32_t numSteps,
                                                  uint16_t stepZeroDelay, uint16_t endVelocityDelay);
static status_t    allocate_accel_table(uint32_t tableSize, int8_t* poolIndex);
static void        free_accel_table(int8_t poolIndex);
static status_t    start_steps(STP_StepperHandle_t* handle);
static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_StepperHandle_t** handle);
static status_t    reset_movement_handle(STP_StepperMovementHandle_t* handle);
static void        process_movement(void);
static void        process_config(void);
static status_t    init_dir_pin(STP_StepperHandle_t* handle);
static status_t    set_direction_pin(STP_StepperHandle_t* handle);
static status_t    init_step_pin(STP_StepperHandle_t* handle);
static inline void calculate_step_profile(STP_StepperHandle_t* handle);
static inline void calculate_delays(STP_StepperHandle_t* handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay);
static inline void calculate_acceleration_table_parameters(STP_StepperHandle_t* handle,
                                                           uint32_t*            interpFactor,
                                                           uint32_t*            tableSize);
static status_t    create_accel_table(STP_StepperHandle_t* handle, uint32_t tableSize,
                                      uint16_t stepZeroDelay, uint16_t endVelocityDelay,
                                      int8_t* poolIndex);
static inline void update_isr_handle_cache(void);

/****************************
 *     Public Variables     *
 ****************************/

STP_HandlesArrayItem_t handles[STP_MAX_HANDLE_COUNT];

/*****************************
 *     Private Variables     *
 *****************************/

STP_StepperHandle_t testHandler;

QueueHandle_t cmdQueue    = NULL;
QueueHandle_t configQueue = NULL;

STP_StepperHandle_t* FTM3_ISR_handle_cache[8];

static uint8_t isrDebugPin = 1;
static GPIO_Type* isrDebugGPIO = GPIOA;
static PORT_Type* isrDebugPort = PORTA;


/**
 * @brief Static pool of acceleration lookup tables.
 * Shared by all stepper motor instances to avoid dynamic allocation.
 */
static STP_AccelTablePoolItem_t accelTablePool[STP_ACCEL_TABLE_POOL_SIZE];

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

    cmdQueue = xQueueCreate(STP_MOVEMENT_QUEUE_SIZE, sizeof(STP_CmdQueueItem_t));
    if (cmdQueue == NULL)
    {
        return kStatus_Fail;
    }
    configQueue = xQueueCreate(STP_CONFIG_QUEUE_SIZE, sizeof(STP_StepperConfig_t));
    if (configQueue == NULL)
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

    return kStatus_Success;
}

void STP_task(void* pvParameters)
{
    LOG_INFO("Started Step Task");
    for (;;)
    {
        process_config();
        process_movement();
        vTaskDelay(1);
    }
}

status_t STP_stop(STP_StepperHandle_t* handle, uint8_t doDeceleration)
{
    STP_CmdQueueItem_t queueItem;
    queueItem.handle                   = handle;
    queueItem.data.stop.doDeceleration = doDeceleration;
    queueItem.type                     = STP_CMD_STOP;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_WARN("Failed to queue stop command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t STP_move_relative(STP_StepperHandle_t* handle, uint32_t stepCount,
                           STP_Direction_t direction)
{
    STP_CmdQueueItem_t queueItem;
    queueItem.handle              = handle;
    queueItem.type                = STP_CMD_MOVE;
    queueItem.data.move.stepCount = stepCount;
    queueItem.data.move.direction = direction;

    if (xQueueSend(cmdQueue, (void*)&queueItem, portMAX_DELAY) != pdTRUE)
    {
        LOG_ERROR("Failed to queue movement command");
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t STP_init_stepper(STP_StepperConfig_t config)
{
    if (xQueueSend(configQueue, (void*)&config, portMAX_DELAY) != pdTRUE)
    {
        LOG_ERROR("Failed to queue stepper config command");
        return kStatus_Fail;
    }
    char logMsg[60];
    snprintf(logMsg, sizeof(logMsg), "[%s] Stepper config queued", config.label);
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

status_t STP_get_handle_by_id(const char* label, STP_StepperHandle_t** handle)
{
    if (label == NULL)
        return kStatus_Fail;
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

/********************************************
 *     Private Function Implementations	   *
 ********************************************/

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
        STP_StepperHandle_t* stepperHandle;

        if (intStatus & (0b1 << i))
        {
            stepperHandle = FTM3_ISR_handle_cache[i];

            stepperHandle->movementHandle.currStepCount++;
            uint32_t currentStepCount   = stepperHandle->movementHandle.currStepCount;
            uint32_t totalSteps         = stepperHandle->movementHandle.totalSteps;
            uint32_t accelSteps         = stepperHandle->movementHandle.accelSteps;
            uint32_t constVelocitySteps = stepperHandle->movementHandle.endVelocitySteps;

            if (currentStepCount > totalSteps)
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

static void process_config(void)
{
    STP_StepperConfig_t queueItem;
    BaseType_t          status;
    status = xQueueReceive(configQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }

    LOG_DEBUG("Processing stepper configuration");
    init_handle(&queueItem);
}

static status_t init_handle(const STP_StepperConfig_t* config)
{
    if (config == NULL)
    {
        return kStatus_Fail;
    }

    STP_StepperHandle_t* handle;
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

    handle->ftmBase    = config->ftmBase;
    handle->ftmChannel = config->ftmChannel;

    handle->stepPort    = config->stepPort;
    handle->stepGPIO    = config->stepGPIO;
    handle->stepPin     = config->stepPin;
    handle->stepMuxFTM  = config->stepMuxFTM;
    handle->stepMuxGPIO = config->stepMuxGPIO;

    handle->dirPort               = config->dirPort;
    handle->dirGPIO               = config->dirGPIO;
    handle->dirPin                = config->dirPin;
    handle->dirLogicHighClockwise = config->dirLogicHighClockwise;

    handle->acceleration = config->acceleration;
    handle->endVelocity  = config->endVelocity;
    handle->label        = config->label;
    handle->dirMux       = config->dirMux;


    if (reset_movement_handle(&handle->movementHandle) != kStatus_Success)
        return kStatus_Fail;

    if (init_step_pin(handle) != kStatus_Success)
        return kStatus_Fail;

    if (init_dir_pin(handle) != kStatus_Success)
        return kStatus_Fail;

    update_isr_handle_cache();

    static char logMsg[90];

    snprintf(logMsg, sizeof(logMsg), "[%s] stepper handle initialized: accel=%.1f, endVel=%.1f",
             handle->label, handle->acceleration, handle->endVelocity);
    LOG_INFO(logMsg);
    return kStatus_Success;
}

static status_t init_step_pin(STP_StepperHandle_t* handle)
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

static status_t init_dir_pin(STP_StepperHandle_t* handle)
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

static status_t step_pin_mux_gpio(STP_StepperHandle_t* handle)
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

static status_t step_pin_mux_ftm(STP_StepperHandle_t* handle)
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

static status_t plan_motion_profile(STP_StepperHandle_t* handle)
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
        return kStatus_Fail;

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

static inline void calculate_step_profile(STP_StepperHandle_t* handle)
{
    uint32_t accelSteps =
        (uint32_t)((handle->endVelocity * handle->endVelocity) /
                   (2 * handle->acceleration));

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

static inline void calculate_delays(STP_StepperHandle_t* handle, uint16_t* stepZeroDelay,
                                    uint16_t* endVelocityDelay)
{
    *stepZeroDelay =
        (uint16_t)(0.676 * STP_TIMER_FREQ_HZ *
                   sqrt(2.0 / (handle->acceleration)));

    *endVelocityDelay = (uint16_t)(STP_TIMER_FREQ_HZ /
                                   (handle->endVelocity));

    handle->movementHandle.endVelocityDelay = *endVelocityDelay;
}

static inline void calculate_acceleration_table_parameters(STP_StepperHandle_t* handle,
                                                           uint32_t*            interpFactor,
                                                           uint32_t*            tableSize)
{
    *interpFactor = STP_ACCEL_TABLE_INTERP_FACTOR;

    if (*interpFactor < 1)
        *interpFactor = 1;

    if (handle->movementHandle.accelSteps > STP_MAX_ACCEL_TABLE_SIZE * (*interpFactor))
    {
        *interpFactor = (handle->movementHandle.accelSteps + STP_MAX_ACCEL_TABLE_SIZE - 1) /
                        STP_MAX_ACCEL_TABLE_SIZE;
    }

    *tableSize = (handle->movementHandle.accelSteps + *interpFactor - 1) / *interpFactor;

    if (*tableSize > STP_MAX_ACCEL_TABLE_SIZE)
        *tableSize = STP_MAX_ACCEL_TABLE_SIZE;
}

static status_t create_accel_table(STP_StepperHandle_t* handle, uint32_t tableSize,
                                   uint16_t stepZeroDelay, uint16_t endVelocityDelay,
                                   int8_t* poolIndex)
{

    if (handle == NULL || poolIndex == NULL)
        return kStatus_Fail;

    status_t status;

    status = allocate_accel_table(tableSize, poolIndex);

    if (status != kStatus_Success)
        return kStatus_Fail;

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
        STP_StepperHandle_t* stepperHandle;
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

static status_t start_steps(STP_StepperHandle_t* handle)
{

    static char logMsg[100];

    if (step_pin_mux_ftm(handle) != kStatus_Success)
        return kStatus_Fail;

    if (set_direction_pin(handle) != kStatus_Success)
        return kStatus_Fail;

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

    handle->movementHandle.state = STP_MOVEMENT_STARTED;

    snprintf(logMsg, sizeof(logMsg), "[%s] Movement started: %lu steps", handle->label,
             handle->movementHandle.totalSteps);
    LOG_DEBUG(logMsg);
    return kStatus_Success;
}

static inline void get_handle_from_timer(FTM_Type* ftmBase, ftm_chnl_t ftmChannel,
                                         STP_StepperHandle_t** handle)
{
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].handle.ftmBase == ftmBase && handles[i].handle.ftmChannel == ftmChannel)
        {
            *handle = &handles[i].handle;
            return;
        }
    }
    *handle = NULL;
    return;
}

static status_t reset_movement_handle(STP_StepperMovementHandle_t* handle)
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

    return kStatus_Success;
}

static status_t stop_steps(STP_StepperHandle_t* handle, uint8_t doDeceleration)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    if (doDeceleration == 0)
    {
        NVIC_DisableIRQ(FTM3_IRQn);
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &=
            ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);
        handle->ftmBase->CONTROLS[handle->ftmChannel].CnSC &= ~(FTM_CnSC_CHF_MASK);
        handle->movementHandle.state = STP_MOVEMENT_STOPPED;
        if (step_pin_mux_gpio(handle) != kStatus_Success)
            return kStatus_Fail;
        NVIC_EnableIRQ(FTM3_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(FTM3_IRQn);
        if (handle->movementHandle.state == STP_MOVEMENT_ACCELERATING)
        {
            handle->movementHandle.isTrapezoidal    = 0;
            handle->movementHandle.endVelocitySteps = 0;
            handle->movementHandle.accelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.decelSteps       = handle->movementHandle.currStepCount;
            handle->movementHandle.totalSteps       = handle->movementHandle.currStepCount * 2;
        }
        else if (handle->movementHandle.state == STP_MOVEMENT_CONST_VELOCITY)
        {
            handle->movementHandle.endVelocitySteps =
                handle->movementHandle.currStepCount - handle->movementHandle.accelSteps;
            handle->movementHandle.totalSteps = handle->movementHandle.accelSteps +
                                                handle->movementHandle.endVelocitySteps +
                                                handle->movementHandle.decelSteps;
        }
        NVIC_EnableIRQ(FTM3_IRQn);
    }

    return kStatus_Success;
}

static status_t set_direction_pin(STP_StepperHandle_t* handle)
{
    if (handle == NULL)
    {
        return kStatus_Fail;
    }

    uint8_t pinValue;
    if (handle->dirLogicHighClockwise)
        pinValue = handle->movementHandle.direction == STP_CLOCKWISE;
    else
        pinValue = handle->movementHandle.direction == STP_COUNTERCLOCKWISE;
    GPIO_PinWrite(handle->dirGPIO, handle->dirPin, pinValue);

    return kStatus_Success;
}

static void process_movement(void)
{
    static char logMsg[80];

    // Check all handles for finished movements
    for (size_t i = 0; i < STP_MAX_HANDLE_COUNT; i++)
    {
        if (handles[i].used == 1 &&
            (handles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED ||
             handles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED))
        {

            step_pin_mux_gpio(&handles[i].handle);

            if (handles[i].handle.movementHandle.state == STP_MOVEMENT_FINISHED)
                snprintf(logMsg, sizeof(logMsg), "[%s] Movement finished", handles[i].handle.label);
            else if (handles[i].handle.movementHandle.state == STP_MOVEMENT_STOPPED)
                snprintf(logMsg, sizeof(logMsg), "[%s] Movement stopped", handles[i].handle.label);
            LOG_DEBUG(logMsg);
            reset_movement_handle(&handles[i].handle.movementHandle);
        }
    }

    STP_CmdQueueItem_t queueItem;
    BaseType_t         status;
    status = xQueueReceive(cmdQueue, &queueItem, pdMS_TO_TICKS(10));
    if (status != pdPASS)
    {
        return;
    }
    switch (queueItem.type)
    {
        case STP_CMD_MOVE:
        {
            LOG_DEBUG("Processing movement command");
            queueItem.handle->movementHandle.totalSteps = queueItem.data.move.stepCount;
            queueItem.handle->movementHandle.direction  = queueItem.data.move.direction;
            if (plan_motion_profile(queueItem.handle) == kStatus_Success)
            {
                start_steps(queueItem.handle);
            }
            break;
        }
        case STP_CMD_STOP:
        {
            LOG_DEBUG("Processing stop command");
            stop_steps(queueItem.handle, queueItem.data.stop.doDeceleration);
            break;
        }
    }
}
