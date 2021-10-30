/*
 * ACME_3416.c
 *
 *  Created on: 12 jun. 2021
 *      Author: Embedded Systems UMA
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/i2c_driver.h"

#define ACME_I2C_ADDRESS 0x3C

#define ACME_ID_REGISTER 0
#define ACME_DIR_REGISTER 1
#define ACME_OUTPUT_REGISTER 2
#define ACME_INT_TYPE_REGISTER_A 3
#define ACME_INT_TYPE_REGISTER_B 4
#define ACME_INT_CLEAR_REGISTER 5
#define ACME_INPUT_REGISTER 6
#define ACME_INT_STATUS_REGISTER 7

#define ACME_ID_REGISTER_VALUE 0xB6

//INICIALIZA EL DISPOSITIVO
//Esta funcion debe comprobar si el sensor esta o no disponible y habilitar las salidas que indiquemos
int32_t ACME_initDevice(void)
{
    uint8_t array_envio[2];
    uint8_t rxvalue;
    int32_t status;

    array_envio[0]=ACME_ID_REGISTER;
    status=I2CDriver_WriteAndRead(ACME_I2C_ADDRESS,array_envio,1,&rxvalue,1);
    if ((status!=0)||(rxvalue!=ACME_ID_REGISTER_VALUE))
            return -1;

    array_envio[0]=ACME_DIR_REGISTER;
    array_envio[1]=0x0F;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    if (status!=0)
       return -1;

    array_envio[0]=ACME_INT_TYPE_REGISTER_A;
    array_envio[1]=0;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    if (status!=0)
            return -1;

    array_envio[0]=ACME_INT_TYPE_REGISTER_B;
    array_envio[1]=0;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    if (status!=0)
            return -1;

    array_envio[0]=ACME_INT_CLEAR_REGISTER;
    array_envio[1]=0x3F;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    if (status!=0)
            return -1;


    return status;
}

//Establece la direccion de los pines
int32_t ACME_setPinDir (uint8_t dir)
{
    uint8_t array_envio[2];
    int32_t status;

    array_envio[0]=ACME_DIR_REGISTER;
    array_envio[1]=dir;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    return status;
}

//Escribe en los pines de salida
int32_t ACME_writePin (uint8_t pin)
{
    uint8_t array_envio[2];
    int32_t status;

    array_envio[0]=ACME_OUTPUT_REGISTER;
    array_envio[1]=pin;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    return status;
}

//Lee los pines de entrada
int32_t ACME_readPin (uint8_t *pin)
{
    uint8_t dir_register=ACME_INPUT_REGISTER;
    int32_t status;

    status=I2CDriver_WriteAndRead(ACME_I2C_ADDRESS,&dir_register,sizeof(dir_register),pin,sizeof(uint8_t));
    return status;
}

//Borra las interrupciones
int32_t ACME_clearInt (uint8_t pin)
{
    uint8_t array_envio[2];
    int32_t status;

    array_envio[0]=ACME_INT_CLEAR_REGISTER;
    array_envio[1]=pin;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    return status;
}

//habilita/deshabilita las intertupciones y el tipo
int32_t ACME_setIntType (uint8_t regA,uint8_t regB)
{



    uint8_t array_envio[3];
    int32_t status;

    array_envio[0]=ACME_INT_TYPE_REGISTER_A;
    array_envio[1]=regA;
    array_envio[2]=regB;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,array_envio,sizeof(array_envio));
    return status;
}

//Lee el estado de interrupción
int32_t ACME_readInt (uint8_t *pin)
{
    uint8_t dir_register=ACME_INT_STATUS_REGISTER;
    int32_t status;

    status=I2CDriver_WriteAndRead(ACME_I2C_ADDRESS,&dir_register,sizeof(dir_register),pin,sizeof(uint8_t));
    return status;
}

//Funcion para probar la lectura...
int32_t ACME_read (uint8_t *bytes,uint8_t size)
{
    int32_t status;
    status=I2CDriver_Read(ACME_I2C_ADDRESS,bytes,size);
    return status;
}
int32_t ACME_write (uint8_t *bytes,uint8_t size)
{
    int32_t status;
    status=I2CDriver_Write(ACME_I2C_ADDRESS,bytes,size);
    return status;
}
