/*
 * TMC2209.h
 *
 *  Created on: 5 Dec 2025
 *      Author: dg
 */

#ifndef TMC2209_H_
#define TMC2209_H_

#include "stdint.h"
#include "fsl_common.h"

#define TMC_TIMEOUT_MS 1000

#define TMC_GCONF_ADDR 0x00
#define TMC_GCONF_LENGTH 10

#define TMC_GSTAT_ADDR 0x01
#define TMC_GSTAT_LENGTH 3

#define TMC_IFCNT_ADDR 0x02
#define TMC_IFCNT_LENGTH 8

#define TMC_NODECONF_ADDR 0x03
#define TMC_NODECONF_LENGTH 4







#define TMC_CONFIRM_WRITES

typedef enum _TMC_SerialAddress
{
  TMC_SERIAL_ADDRESS_0=0,
  TMC_SERIAL_ADDRESS_1=1,
  TMC_SERIAL_ADDRESS_2=2,
  TMC_SERIAL_ADDRESS_3=3,
} TMC_SerialAddress_t;

typedef enum _TMC_DevStatus {
    TMC_SLAVE_0_ACTIVE = 1,
    TMC_SLAVE_1_ACTIVE = 2,
    TMC_SLAVE_2_ACTIVE = 4,
    TMC_SLAVE_3_ACTIVE = 8
} TMC_SlaveStatus_t;

typedef struct _TMC_GCONF {
    uint8_t I_scale_analog;
    uint8_t  internal_Rsense;
    uint8_t  en_SpreadCycle;
    uint8_t shaft;
    uint8_t index_otpw;
    uint8_t index_step;
    uint8_t pdn_disable;
    uint8_t mstep_reg_select;
    uint8_t multistep_filt ;
} TMC_GCONF_t;

typedef struct _TMC_GSTAT {
    uint8_t reset;
    uint8_t drv_err;
    uint8_t uv_cp;
} TMC_GSTAT_t;

typedef struct _TMC_IOIN {
    uint8_t ENN;
    uint8_t MS1;
    uint8_t MS2;
    uint8_t DIAG;
    uint8_t PDN_UART;
    uint8_t STEP;
    uint8_t SPREAD_EN;
    uint8_t DIR;
    uint16_t VERSION;
} TMC_IOIN_t;

typedef struct _TMC2209_Write_Datagram{
    uint8_t sync: 4;
    uint8_t reserved: 4;
    uint8_t dev_addr: 8;
    uint8_t reg_addr: 8;
    uint32_t data: 32;
    uint8_t crc: 8;
} TMC2209_Write_Datagram_t;

typedef struct _TMC2209_Read_Datagram{
    uint8_t sync: 4;
    uint8_t reserved: 4;
    uint8_t dev_addr: 8;
    uint8_t reg_addr: 8;
    uint8_t crc: 8;
}TMC2209_Read_Datagram_t ;

status_t TMC_Init(void);
status_t TMC_GetSlaveStates(TMC_SlaveStatus_t *slaveStatus);
status_t TMC_read(TMC_SerialAddress_t serialAdress, uint8_t regAddr, uint32_t *data);
status_t  TMC_write(TMC_SerialAddress_t serialAdress, uint8_t regAddr, uint32_t *data);
status_t TMC_Read_TransmissionCount(TMC_SerialAddress_t serialAdress,
         uint8_t * transmissionCount);

#endif /* TMC2209_H_ */
