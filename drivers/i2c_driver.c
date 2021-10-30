/*
 * i2c_driver.c
 *
 *  Created on: 1 may. 2021
 *      Author: Embedded Systems UMA
 */

//Adaptada a FreeRTOS por J. M. Cano,  E Gonzalez e Ignacio Herrero
// Para mejorar la eficiencia gestion del bus I2C aprovechando el RTOS se utilizan los siguientes mecanismos:
// -> Las transacciones I2C las realiza una rutina de interrupciï¿½n
// -> Las funciones I2C_IF_Read, I2C_IF_ReadFrom e I2C_IF_Write envian los datos de la transaccion a realizar mediante una cola de mensaje.
// -> La ISR va procesando las peticiones de transaccion/realizando nuevas transacciones. Cuando finaliza cada transaccion utiliza las DirectToTaskNotification para desbloquear la tarea.
// Se mantiene la compatibilidad hacia atras, por eso las funciones de las bibliotecas bma222drv.c y tmp000drv.c no hay que cambiarlas.

//2018. Adaptado de la CC3200 a la TIVA.
// --> La TIVA no tiene flag de interrupcion por error y otras causas (NACK), hay que tratarlo de otra manera (Mediante una funcion que comrprueba si ha habido error).
// --> Arreglado un fallo por el cual no saltaba la notificaciï¿½n directa a tarea en el caso de este error.

// Standard includes


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// Common interface include
#include "i2c_driver.h"

//Include FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "portmacro.h"
#include "event_groups.h"

#define ACME_EMULATION_MODE 1

#if ACME_EMULATION_MODE
//Habilitar para emulacion dispositivo ACME...
#include "emul/ACMESim_i2c_slave.h"
#endif


//*****************************************************************************
//
// I2C transaction time-out value.
// Set to value 0x7D. (@100KHz it is 20ms, @400Khz it is 5 ms)
//
//*****************************************************************************
#define I2CDRIVER_TIMEOUT_VAL         0x7D


//Datos de la transaccion a realizar/en curso.
typedef struct {
    TaskHandle_t mejora;
    uint8_t *txbuffer;    /* puntero a los datos TX/RX */
    uint8_t *rxbuffer;    /* puntero a los datos RX */
    uint8_t rxlenght;   /* longitud a recibir */
    uint8_t txlenght;   /* longitud a transmitir */
    uint8_t command;    /* comando */
    uint8_t dev_address; /* direccion I2C */
} I2C_Transaction;



//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define SYS_CLK                 configCPU_CLOCK_HZ
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//ISR states....
#define I2C_ISR_MASTER_STATE_IDLE              0
#define I2C_ISR_MASTER_STATE_WRITE_NEXT        1
#define I2C_ISR_MASTER_STATE_WRITE_FINAL       2
#define I2C_ISR_MASTER_STATE_READ_NEXT         3
#define I2C_ISR_MASTER_STATE_READ_FINAL        4

//Comandos (tipos de transaccion) que se pueden realizar
#define I2C_COMMAND_WRITE 0
#define I2C_COMMAND_READ 1
#define I2C_COMMAND_WRITE_READ 2


//Flags para las DirectToTaskNotifications
#define I2C_NOTIFY_READ_COMPLETE (0x01)
#define I2C_NOTIFY_WRITE_COMPLETE (0x02)
#define I2C_NOTIFY_ERR (0x04)


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

//Controla los estados que atraviesa la ISR mientras se va realizando la transaccion
//De esta forma cada vez que salta la ISR se ejecuta un comportamiento que depende de su estado
static volatile uint16_t g_i2cisrstate=I2C_ISR_MASTER_STATE_IDLE;
static QueueHandle_t g_I2C_transaction_queue;







//****************************************************************************
//
//! Invokes the I2C driver APIs to write to the specified address
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the data to be written
//! \param ucLen is the length of data to be written
//!
//! This function works in a polling mode,
//!    1. Writes the device register address to be written to.
//!    2. In a loop, writes all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int32_t I2CDriver_Write(uint8_t ucDevAddr,
                           uint8_t *pucData,
                           uint8_t ucLen)
{
    uint32_t notifVal=0;
    I2C_Transaction transaction;

    RETERR_IF_TRUE(pucData == NULL);
    RETERR_IF_TRUE(ucLen == 0);


    transaction.mejora=xTaskGetCurrentTaskHandle();
    transaction.txbuffer=pucData;
    transaction.rxbuffer=NULL;
    transaction.txlenght=ucLen;
    transaction.rxlenght=0;
    transaction.dev_address=ucDevAddr;
    transaction.command=I2C_COMMAND_WRITE;

    //Envia la transaccion a la cola de mensajes...
    xQueueSend(g_I2C_transaction_queue,&transaction,portMAX_DELAY);

    if (g_i2cisrstate==I2C_ISR_MASTER_STATE_IDLE)
    {
        IntPendSet(INT_I2C1);   //Produce un disparo software de la ISR (comienza a transmitir)....
    }

    while (!(notifVal&(I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR)))
        {
            xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
        }

        if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

        return SUCCESS;

}


int32_t I2CDriver_Read(uint8_t ucDevAddr,
                          uint8_t *pucData,
                          uint8_t ucLen)
{
    //unsigned long ulCmdID;
    uint32_t notifVal=0;
    I2C_Transaction transaction;

    RETERR_IF_TRUE(pucData == NULL);
    RETERR_IF_TRUE(ucLen == 0);


    transaction.mejora=xTaskGetCurrentTaskHandle();
    transaction.rxbuffer=pucData;
    transaction.txbuffer=NULL;
    transaction.txlenght=0;
    transaction.rxlenght=ucLen;
    transaction.dev_address=ucDevAddr;
    transaction.command=I2C_COMMAND_READ;

    xQueueSend(g_I2C_transaction_queue,&transaction,portMAX_DELAY);

    if (g_i2cisrstate==I2C_ISR_MASTER_STATE_IDLE)
    {
        IntPendSet(INT_I2C1);   //Produce un disparo software de la interrupción (comienza a transmitir)....
    }

    while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
        {
            xTaskNotifyWait( 0, I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
        }

        if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

        return SUCCESS;
}


int32_t I2CDriver_WriteAndRead(uint8_t ucDevAddr,
                               uint8_t *pucWrDataBuf,
                               uint8_t ucWrLen,
                               uint8_t *pucRdDataBuf,
                               uint8_t ucRdLen)
{

    uint32_t notifVal=0;
    volatile int i=0;
    I2C_Transaction transaction;


        RETERR_IF_TRUE(pucRdDataBuf == NULL);
        RETERR_IF_TRUE(pucWrDataBuf == NULL);
        RETERR_IF_TRUE(ucWrLen == 0);
        RETERR_IF_TRUE(ucWrLen > ucWrLen);

        transaction.mejora=xTaskGetCurrentTaskHandle();
        transaction.txbuffer=pucWrDataBuf;
        transaction.rxbuffer=pucRdDataBuf;
        transaction.txlenght=ucWrLen;
        transaction.rxlenght=ucRdLen;
        transaction.dev_address=ucDevAddr;
        transaction.command=I2C_COMMAND_WRITE_READ;

        xQueueSend(g_I2C_transaction_queue,&transaction,portMAX_DELAY);

        if (g_i2cisrstate==I2C_ISR_MASTER_STATE_IDLE)
        {
            IntPendSet(INT_I2C1);   //Produce un disparo software....
        }


        //CEspera a que se complete la operacion de escritura/lectura o se produza error
               while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
               {
                   xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
                   i++;
               }

               if (notifVal&I2C_NOTIFY_ERR)
               {
                   return FAILURE;
               }

           return SUCCESS;
}


//Rutina de interrupcion.
//Esta rutina parece muy larga, pero sï¿½lo se ejecuta una parte u otra segï¿½n el estado en el que estemos...
//Utiliza una mï¿½quina de estados para cambiar el comportamiento cuando se produce la interrupcion, ya que lo que se debe realizar depende de si estamos o no
// en una transacciï¿½n, del tipo de transaccion (escritura, lectura o escritura-lectura, y de que punto de dicha transacciï¿½n estamos.
// Para ello se utiliza la variable de estado g_i2cisrstate.

void I2CDriver_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;

    //The following two variables should maintain values between calls to the ISR ---> they are local statics
    static I2C_Transaction transaction;
    static uint8_t *tmpptr;

#if ACME_EMULATION_MODE
    //Esto es necesario para la emulacion del dispositivo ACME.... NO QUITAR!!!
    if (I2CSlaveIntStatus(I2C1_BASE, true) )
    {
        ACMEsim_I2C1SlaveIntHandler();
    }
#endif

    if (I2CMasterIntStatus (I2C1_BASE,1)||(g_i2cisrstate==I2C_ISR_MASTER_STATE_IDLE))
    {
        I2CMasterIntClear(I2C1_BASE); //Borra el flag de interrupcion

        //Primero tratamos posible error... (por ejemplo NACK,)
        //No se implementa el error de timeout por exceso de tiempo con CLK a 0  (no configurado).
        if (I2CMasterErr(I2C1_BASE)&&(g_i2cisrstate!=I2C_ISR_MASTER_STATE_IDLE))
        {
            I2CMasterIntDisable(I2C1_BASE);

            switch (g_i2cisrstate)
            {
            case I2C_ISR_MASTER_STATE_READ_NEXT:
            case I2C_ISR_MASTER_STATE_READ_FINAL:
                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
                break;
            case I2C_ISR_MASTER_STATE_WRITE_NEXT:
            case I2C_ISR_MASTER_STATE_WRITE_FINAL:
                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
                break;
            }
            g_i2cisrstate=I2C_ISR_MASTER_STATE_IDLE;

            xTaskNotifyFromISR(transaction.mejora,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
            xQueueReceiveFromISR(g_I2C_transaction_queue,&transaction,&xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);    //Esto es necesario antes del return...
            return; //Esto finaliza la ISR
        }


        //Ejecuta la maquina de estados para responder al evento. Miramos primero en que estado estamos.
        switch(g_i2cisrstate)
        {
            case I2C_ISR_MASTER_STATE_IDLE:
            {
                //Arranca la transaccion....
                if (xQueuePeekFromISR(g_I2C_transaction_queue,&transaction)==pdTRUE)
                {

                    //Hay algo en la cola... puedo comenzar
                    I2CMasterIntEnable(I2C1_BASE);
                    switch (transaction.command)
                    {
                        /* TRANSACCION WRITE */
                        case I2C_COMMAND_WRITE:

                        {   //Disparo una escritura
                            tmpptr=transaction.txbuffer;

                            I2CMasterSlaveAddrSet(I2C1_BASE, transaction.dev_address, false); // Set I2C codec slave address
                            I2CMasterDataPut(I2C1_BASE, *tmpptr);    // Write the first byte to the controller.
                            I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_SEND_START);

                            transaction.txlenght--;
                            tmpptr++;

                            if (transaction.txlenght>0)
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_WRITE_NEXT; //Cambia de estado. La proxima interrupcion Escribe otro caracter
                            else
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_WRITE_FINAL; //Cambia de estado. La proxima interrupcion finaliza la transmision

                        }
                        break;
                        /* TRANSACCION READ o COMBINADA */
                        case I2C_COMMAND_WRITE_READ:{

                            tmpptr=transaction.txbuffer;

                            I2CMasterSlaveAddrSet(I2C1_BASE, transaction.dev_address, false); // Set I2C codec slave address
                            I2CMasterDataPut(I2C1_BASE, *tmpptr);    // Write the first byte to the controller.
                            I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_SEND_START);

                            transaction.txlenght--;
                            tmpptr++;

                            if (transaction.txlenght>0)
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_WRITE_NEXT; //Cambia de estado. La proxima interrupcion Escribe otro caracter
                            else
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_WRITE_FINAL;

                        }
                        break;
                        case I2C_COMMAND_READ:

                            tmpptr=transaction.rxbuffer;

                            I2CMasterSlaveAddrSet(I2C1_BASE, transaction.dev_address, true);


                            transaction.rxlenght--;
                            if(transaction.rxlenght==0){
                                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_READ_FINAL; //final de lectura


                            }else{
                                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_START);
                                g_i2cisrstate=I2C_ISR_MASTER_STATE_READ_NEXT;


                            }
                        break;
                    }
                    //Habilitamos las interrupciones, para que salten conforme se van completando

                }
                else
                {
                    //No habia nada en la cola de ordenes I2C
                    //¿Que ha pasao??
                   // I2CMasterIntDisable(I2C1_BASE);
                }
            }
            break; //FIN DEL ESTADO I2C_ISR_MASTER_STATE_IDLE...

            case I2C_ISR_MASTER_STATE_WRITE_NEXT:
            {   //Continuacion de escritura...Envio el siguiente byte
                I2CMasterDataPut(I2C1_BASE, *tmpptr);
                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_SEND_CONT);

                transaction.txlenght--;
                tmpptr++;
                if (transaction.txlenght<=0) g_i2cisrstate=I2C_ISR_MASTER_STATE_WRITE_FINAL; //Si no hay mas que enviar, la siguiente ISR finaliza la transmision (condicion de STOP)
            }

            break; //FIN DEL ESTADO I2C_ISR_MASTER_STATE_WRITE_NEXT

            case I2C_ISR_MASTER_STATE_WRITE_FINAL:
            {   //Fin transmision. Si era una transmision simple, finalizo y paso a la siguiente
                if (transaction.command!=I2C_COMMAND_WRITE_READ)
                {
                    //Transaccion finalizada
                    //Deshabilito las ISR, vuelvo al estado IDLE, elimino la transaccion de la cola
                    //Finalmente compruebo si hay almacenada en la cola para dispararla
                    I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_SEND_STOP);
                    I2CMasterIntDisable(I2C1_BASE);

                    g_i2cisrstate=I2C_ISR_MASTER_STATE_IDLE; //Vuelve al estado IDLE

                    //Hemos finalizado
                    xTaskNotifyFromISR(transaction.mejora,I2C_NOTIFY_WRITE_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);    //Transaccion correcta
                    xQueueReceiveFromISR(g_I2C_transaction_queue,&transaction,&xHigherPriorityTaskWoken); //Se elimina de la cola para permitir que ejecute la siguiente

                }
                else //Si era la operacion WRITE_READ...
                {
                    tmpptr=transaction.rxbuffer;
                    I2CMasterSlaveAddrSet(I2C1_BASE, transaction.dev_address, true);
                    transaction.rxlenght--;
                    if(transaction.rxlenght==0)
                    {
                        I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
                        g_i2cisrstate=I2C_ISR_MASTER_STATE_READ_FINAL;
                    }else{
                        I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_START);
                        g_i2cisrstate=I2C_ISR_MASTER_STATE_READ_NEXT;
                    }

               }
            }
            break; //FIN DEL ESTADO I2C_ISR_MASTER_STATE_WRITE_FINAL

            case I2C_ISR_MASTER_STATE_READ_NEXT:
            {   //Lectura "larga" en curso... Intento leer datos  continuar...
                *tmpptr = I2CMasterDataGet(I2C1_BASE);
                 transaction.rxlenght--;
                 tmpptr++;
                 if (transaction.rxlenght==0){
                     I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                     g_i2cisrstate=I2C_ISR_MASTER_STATE_READ_FINAL;
                 }
                 //seguimos leyendo
                 else{
                     I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                 }
            }
            break; //FIN DEL ESTADO I2C_ISR_MASTER_STATE_READ_NEXT

            case I2C_ISR_MASTER_STATE_READ_FINAL:
            {
                *tmpptr = I2CMasterDataGet(I2C1_BASE);
                I2CMasterIntDisable(I2C1_BASE);
                g_i2cisrstate=I2C_ISR_MASTER_STATE_IDLE;

                xTaskNotifyFromISR(transaction.mejora,I2C_NOTIFY_READ_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);    //Transaccion correcta
                xQueueReceiveFromISR(g_I2C_transaction_queue,&transaction,&xHigherPriorityTaskWoken); //Se elimina de la cola para permitir que ejecute la siguiente

            }
            break; //FIN DEL ESTADO I2C_ISR_MASTER_STATE_READ_FINAL
        }
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


//****************************************************************************
//
//! Enables and configures the I2C driver
//!
//! \param ulMode is the mode configuration of I2C
//! The parameter \e ulMode is one of the following
//! - \b I2C_MASTER_MODE_STD for 100 Kbps standard mode.
//! - \b I2C_MASTER_MODE_FST for 400 Kbps fast mode.
//!
//! This function works in a polling mode,
//!    1. Powers ON the I2C peripheral.
//!    2. Configures the I2C peripheral
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int32_t I2CDriver_Open(uint8_t ulMode)
{

    // Inicializacion del interfaz con los sensores
    //
    // The I2C1 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7,
                            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
    //
    // Configure the pin muxing for I2C1 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    //por el clock gating
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C1);



    //
    // Configure I2C module in the specified mode
    //
    switch(ulMode)
    {
        case I2CDRIVER_MASTER_MODE_STD:       /* 100000 */
            I2CMasterInitExpClk(I2C1_BASE,SYS_CLK,false);
            break;

        case I2CDRIVER_MASTER_MODE_FST:       /* 400000 */
            I2CMasterInitExpClk(I2C1_BASE,SYS_CLK,true);
            break;

        default:
            I2CMasterInitExpClk(I2C1_BASE,SYS_CLK,true);
            break;
    }

    //Empezamos por estado IDLE (no hay transaccion en marcha)
    g_i2cisrstate=I2C_ISR_MASTER_STATE_IDLE;

    g_I2C_transaction_queue=xQueueCreate(1,sizeof(I2C_Transaction));
        if (g_I2C_transaction_queue==NULL)
            while(1);





    IntPrioritySet(INT_I2C1,configMAX_SYSCALL_INTERRUPT_PRIORITY); //jose: La prioridad debe ser mayor o igual que configMAX_SYSCALL_INTERRUPT_PRIORITY
    IntEnable(INT_I2C1);
    I2CLoopbackEnable(I2C1_BASE);

#if ACME_EMULATION_MODE
    ACMEsim_InitSlave(); //habilita la configuración del dispositivo I2C ACME emulado
#endif

    return SUCCESS;
}

//****************************************************************************
//
//! Disables the I2C peripheral
//!
//! \param None
//!
//! This function works in a polling mode,
//!    1. Powers OFF the I2C peripheral.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
I2CDriver__Close()
{
    //
    // Power OFF the I2C peripheral
    //

    // Debería comprobarse antes si no hay una transaccion en marcha!!!???

    IntDisable(INT_I2C1);   //Deshabilita la ISR
    SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
    if (g_I2C_transaction_queue!=NULL)
    {
            vQueueDelete(g_I2C_transaction_queue);
            g_I2C_transaction_queue=NULL;
    }

    return SUCCESS;
}






