#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"


static QueueHandle_t cola_adc;
static QueueHandle_t cola_adc1;

extern EventGroupHandle_t xCreatedEventGroup;
extern uint16_t BIT_2,BIT_4;
//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC0_BASE,0);

}


void configADC_IniciaADC(void)
{
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC1);

				//HABILITAMOS EL GPIOE
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
				SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);
				// Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
				GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
				GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_1);


				//CONFIGURAR SECUENCIADOR 1
				ADCSequenceEnable(ADC0_BASE,0);

				//CONFIGURAR SECUENCIADOR 2
				ADCSequenceEnable(ADC1_BASE,0);


				//Configuramos la velocidad de conversion al maximo (1MS/s)
				ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);


				//ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0);	//Dispari,0,0,ADC_CTL_CH0);
				ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1);
				ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH2);
				ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH3);
				ADCSequenceStepConfigure(ADC0_BASE,0,4,ADC_CTL_CH5);
				ADCSequenceStepConfigure(ADC0_BASE,0,5,ADC_CTL_CH6|ADC_CTL_IE|ADC_CTL_END);
				//ADCSequenceStepConfigure(ADC0_BASE,2,1,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);	//La ultima muestra provoca la interrupcion;


				//ADC1

                ADCSequenceConfigure(ADC1_BASE,0,ADC_TRIGGER_TIMER,0);
                TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
				ADCSequenceStepConfigure(ADC1_BASE,0,0,ADC_CTL_CH0);
                ADCSequenceStepConfigure(ADC1_BASE,0,1,ADC_CTL_CH1);
                ADCSequenceStepConfigure(ADC1_BASE,0,2,ADC_CTL_CH2);
                ADCSequenceStepConfigure(ADC1_BASE,0,3,ADC_CTL_CH3);
                ADCSequenceStepConfigure(ADC1_BASE,0,4,ADC_CTL_CH5);
                ADCSequenceStepConfigure(ADC1_BASE,0,5,ADC_CTL_CH6|ADC_CTL_IE|ADC_CTL_END);
                //ADCSequenceStepConfigure(ADC0_BASE,2,1,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);   //La ultima muestra provoca la interrupcion;


				//Habilita las interrupciones
				ADCIntClear(ADC0_BASE,0);
				ADCIntEnable(ADC0_BASE,0);
				IntPrioritySet(INT_ADC0SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);
				IntEnable(INT_ADC0SS0);


				//ADC1
				ADCIntClear(ADC1_BASE,0);
                ADCIntEnable(ADC1_BASE,0);
                IntPrioritySet(INT_ADC1SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                IntEnable(INT_ADC1SS0);

				/*
				IntPrioritySet(INT_ADC0SS2,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                IntEnable(INT_ADC0SS2);
                IntPrioritySet(INT_ADC0SS3,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                IntEnable(INT_ADC0SS3);*/
				//Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
				cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
				if (cola_adc==NULL)
				{
					while(1);
				}

				cola_adc1=xQueueCreate(8,sizeof(MuestrasADC));
                    if (cola_adc1==NULL)
                    {
                        while(1);
                    }
}


void configADC_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_LeeADC1(MuestrasADC *datos)
{
    xQueueReceive(cola_adc1,datos,portMAX_DELAY);
}

void configADC_ISR(void)
{
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;
	    xEventGroupSetBitsFromISR(xCreatedEventGroup, BIT_2, &higherPriorityTaskWoken );
	MuestrasLeidasADC leidas;
	MuestrasADC finales;
	ADCIntClear(ADC0_BASE,0);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
	ADCSequenceDataGet(ADC0_BASE,0,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

	finales.chan1=leidas.chan1;
	finales.chan2=leidas.chan2;
	finales.chan3=leidas.chan3;
	finales.chan4=leidas.chan4;
	finales.chan5=leidas.chan5;
	finales.chan6=leidas.chan6;

	//Guardamos en la cola
	xQueueSendFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void configADC1_ISR(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    xEventGroupSetBitsFromISR(xCreatedEventGroup, BIT_4, &higherPriorityTaskWoken );

    MuestrasLeidasADC leidas1;
    MuestrasADC finales1;
    ADCIntClear(ADC1_BASE,0);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDataGet(ADC1_BASE,0,(uint32_t *)&leidas1);//COGEMOS LOS DATOS GUARDADOS

    finales1.chan1=leidas1.chan1;
    finales1.chan2=leidas1.chan2;
    finales1.chan3=leidas1.chan3;
    finales1.chan4=leidas1.chan4;
    finales1.chan5=leidas1.chan5;
    finales1.chan6=leidas1.chan6;

    //Guardamos en la cola
    xQueueSendFromISR(cola_adc1,&finales1,&higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
