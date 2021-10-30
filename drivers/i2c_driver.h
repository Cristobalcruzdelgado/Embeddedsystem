/*
 * i2c_driver.h
 *
 *  Created on: 5 may. 2021
 *      Author: Embedded Systems UMA
 */

#ifndef DRIVERS_I2C_DRIVER_H_
#define DRIVERS_I2C_DRIVER_H_

#include <stdint.h>

#define I2CDRIVER_MASTER_MODE_STD     0
#define I2CDRIVER_MASTER_MODE_FST     1

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern int32_t I2CDriver_Open(uint8_t ulMode);

extern int32_t I2CDriver_Close();

extern int32_t I2CDriver_Write(uint8_t ucDevAddr,
                           uint8_t *pucData,
                           uint8_t ucLen);
extern int32_t I2CDriver_Read(uint8_t ucDevAddr,
                          uint8_t *pucData,
                          uint8_t ucLen);

extern int32_t I2CDriver_WriteAndRead(uint8_t ucDevAddr,
                               uint8_t *pucWrDataBuf,
                               uint8_t ucWrLen,
                               uint8_t *pucRdDataBuf,
                               uint8_t ucRdLen);

#endif /* DRIVERS_I2C_DRIVER_H_ */
