/************************************************************
 * @file    freertos_heap.c
 * @brief   Application-provided FreeRTOS heap buffer.
 ************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "fsl_common.h"

__attribute__((section(".freeRTOSHeap"))) SDK_ALIGN(uint8_t ucHeap[configTOTAL_HEAP_SIZE], 4);
