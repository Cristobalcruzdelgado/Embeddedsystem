//*****************************************************************************
//
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano, Cristóbal Cruz Delgado
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "drivers/configADC.h"
#include "commands.h"
#include "emul/ACMESim_i2c_slave.h"
#include "drivers/ACME_3416.h"
#include "drivers/i2c_driver.h"

#include <remotelink.h>
#include <serialprotocol.h>

EventGroupHandle_t xCreatedEventGroup;
SemaphoreHandle_t g_pUARTSemaphore;

//parametros de funcionamiento de la tareas
#define REMOTELINK_TASK_STACK (512)
#define REMOTELINK_TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define ADC_TASK_STACK (256)
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY+1)
const uint16_t BIT_0 = 0x0001;
const uint16_t BIT_2 = 0x0002;
const uint16_t BIT_4 = 0x0004;


//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
QueueHandle_t cola;
int contador=0;
bool timer=false;
EventGroupHandle_t xCreatedEventGroup;



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************


//Esta tarea no tendria por que ir en main.c

static portTASK_FUNCTION(EnvioTask,pvParameters)
{
    uint8_t recibido;
    MESSAGE_SONDEO_PARAMETER parametro;
    EventBits_t uxBits;
    MuestrasADC muestras, aux;
        MESSAGE_ADC_SAMPLE_PARAMETER parameter;
        MESSAGE_TIMER_PARAMETER parametro_adc;
       //
       // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
       //
       while(1)
       {


           uxBits =  xEventGroupWaitBits(xCreatedEventGroup,BIT_0|BIT_2|BIT_4,pdTRUE,pdFALSE,portMAX_DELAY);
           if(uxBits&BIT_0)
           {
                if (xQueueReceive(cola,&recibido, portMAX_DELAY)==pdTRUE){
                    //Recibimos valores de la interrupcion empaquetamos y enviamos al PC

                    if(recibido & GPIO_PIN_4){
                        parametro.led_1=1;
                    }else parametro.led_1=0;
                    if(recibido & GPIO_PIN_0){
                        parametro.led_2=1;
                    }else parametro.led_2=0;

                    remotelink_sendMessage(MESSAGE_SONDEO,(void *)&parametro,sizeof(parametro));


                }
                xEventGroupClearBits(
                        xCreatedEventGroup,  /* The event group being updated. */
                                                BIT_0);/* The bits being cleared. */
           }else if(uxBits&BIT_2)
           {

               configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)

                            //Copia los datos en el parametro (es un poco redundante)
                       parameter.chan1=muestras.chan1;
                       parameter.chan2=muestras.chan2;
                       parameter.chan3=muestras.chan3;
                       parameter.chan4=muestras.chan4;
                       parameter.chan5=muestras.chan5;
                       parameter.chan6=muestras.chan6;

                       //Encia el mensaje hacia QT
                       remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&parameter,sizeof(parameter));
                       xEventGroupClearBits(
                                               xCreatedEventGroup,  /* The event group being updated. */
                                                                       BIT_2);/* The bits being cleared. */

           }else if(uxBits&BIT_4)
           {
               configADC_LeeADC1(&aux);    //Espera y lee muestras del ADC (BLOQUEANTE)

                    if(contador<6){
                        parametro_adc.chan1[contador]=aux.chan1;
                        parametro_adc.chan2[contador]=aux.chan2;
                        parametro_adc.chan3[contador]=aux.chan3;
                        parametro_adc.chan4[contador]=aux.chan4;
                        parametro_adc.chan5[contador]=aux.chan5;
                        parametro_adc.chan6[contador]=aux.chan6;
                                        contador++;
                                    }else{
                                        contador=0;
                                        remotelink_sendMessage(MESSAGE_TIMER,(void *)&parametro_adc,sizeof(parametro_adc));
                                        xEventGroupClearBits(
                                                                xCreatedEventGroup,  /* The event group being updated. */
                                                                                        BIT_4);/* The bits being cleared. */
                                    }
           }
    }

}



//Funcion callback que procesa los mensajes recibidos desde el PC (ejecuta las acciones correspondientes a las ordenes recibidas)
static int32_t messageReceived(uint8_t message_type, void *parameters, int32_t parameterSize)
{
    int32_t status=0;   //Estado de la ejecucion (positivo, sin errores, negativo si error)
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    //Comprueba el tipo de mensaje
    switch (message_type)
    {
        case MESSAGE_PING:
        {
            status=remotelink_sendMessage(MESSAGE_PING,NULL,0);
        }
        break;
        case MESSAGE_LED_GPIO:
        {
                MESSAGE_LED_GPIO_PARAMETER parametro;

                if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,parametro.value);
                }
                else
                {
                    status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                }
        }
        break;

        case MESSAGE_ACME:
                {
                        MESSAGE_ACME_PARAMETER parametro;


                        if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                        {
                            ACME_writePin(parametro.gpio0|(parametro.gpio1<<1)|(parametro.gpio2<<2)|(parametro.gpio3<<3));


                        }
                        else
                        {
                            status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                        }
                }
                break;

        case MESSAGE_LED_PWM_BRIGHTNESS:
       {
           MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
               RGBIntensitySet(parametro.rIntensity);
            }
            else
           {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;

        case MESSAGE_FRECUENCIA:
              {
                  MESSAGE_FRECUENCIA_PARAMETER parametro;
                  uint32_t ui16Period;
                   if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                   {
                     // RGBIntensitySet(parametro.frec);
                      ui16Period = SysCtlClockGet()/parametro.frec;
                      TimerLoadSet(TIMER2_BASE, TIMER_A, ui16Period);
                   }
                   else
                  {
                       status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                   }
               }
               break;

        case MESSAGE_PROMEDIO_HARDWARE:
              {
                  MESSAGE_PROMEDIO_HARDWARE_PARAMETER parametro;

                   if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                   {
                       ADCHardwareOversampleConfigure(ADC0_BASE,parametro.Phar);
                   }
                   else
                  {
                       status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                   }
               }
               break;

        case MESSAGE_MODO:
                {
                    MESSAGE_MODO_PARAMETER parametro;
                    if (check_and_extract_command_param(parameters, parameterSize,&parametro, sizeof(parametro))>0)
                        {
                            if (parametro.x == 0){ //Si es 0 activamos el modo GPIO
                                RGBDisable();
                                ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
                            }else if (parametro.x == 1){ //Si es 1 activamos el modo PWM
                                RGBEnable();
                            }
                            return 0;
                        }
                        else
                        {
                            return PROT_ERROR_INCORRECT_PARAM_SIZE;
                        }
                    }
                break;

        case MESSAGE_SONDEO:
        {

            MESSAGE_SONDEO_PARAMETER parametro;
            if (check_and_extract_command_param(parameters, parameterSize,&parametro, sizeof(parametro))>0)
                  {
            parametro.led_1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
            parametro.led_2 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
            remotelink_sendMessage(MESSAGE_SONDEO,(void *)&parametro,sizeof(parametro));

            return 0;
                  }
            else
            {
                return PROT_ERROR_INCORRECT_PARAM_SIZE;
            }
        }

        case MESSAGE_INTERRUPCION:
                        {
                            MESSAGE_INTERRUPCION_PARAMETER parametro;
                            if (check_and_extract_command_param(parameters, parameterSize,&parametro, sizeof(parametro))>0)
                                {
                                    if (parametro.Int == 0){
                                        GPIOIntDisable( GPIO_PORTF_BASE,GPIO_INT_PIN_0| GPIO_INT_PIN_4);

                                    }else if (parametro.Int == 1){
                                        GPIOIntEnable( GPIO_PORTF_BASE,GPIO_INT_PIN_0| GPIO_INT_PIN_4);
                                    }
                                    return 0;
                                }
                                else
                                {
                                    return PROT_ERROR_INCORRECT_PARAM_SIZE;
                                }
                            }
                        break;

        case MESSAGE_COLOR:
                {
                    MESSAGE_COLOR_PARAMETER parametro;
                        uint32_t array[3]={0x0000, 0x0000, 0x0000};
                        if (check_and_extract_command_param(parameters, parameterSize,&parametro, sizeof(parametro))>0)
                        {

                            array[0]=parametro.r <<8;
                            array[1]=parametro.g <<8;
                            array[2]=parametro.b <<8;
                            RGBColorSet(array);
                            return 0;   //Devuelve Ok
                        }
                        else
                            {
                                return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                            }

                        }


                break;

        case MESSAGE_ACTIVA_TIMER:
           {
               MESSAGE_ACTIVA_TIMER_PARAMETER parametro;
               if (check_and_extract_command_param(parameters, parameterSize,&parametro, sizeof(parametro))>0)
                   {
                       if (parametro.Tim == 1){

                           TimerEnable(TIMER2_BASE, TIMER_A);
                           //configADC_DisparaADC();

                       }else if (parametro.Tim == 0){

                           TimerDisable(TIMER2_BASE, TIMER_A);

                       }
                   }


                   else
                   {
                       return PROT_ERROR_INCORRECT_PARAM_SIZE;
                   }
               }
           break;


        case MESSAGE_ADC_SAMPLE:
        {

             //Dispara la conversion (por software)
            configADC_DisparaADC();

        }
        break;
       default:
           //mensaje desconocido/no implementado
           status=PROT_ERROR_UNIMPLEMENTED_COMMAND; //Devuelve error.
    }


    return status;   //Devuelve status
}


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void){

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	MAP_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);


	    //Inicializa el puerto F (LEDs)
	        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	        MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //LEDS APAGADOS

	        //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
	        ButtonsInit();
	        IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);
	        MAP_GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_BOTH_EDGES);
	        MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
	        MAP_IntEnable(INT_GPIOF);
	        GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);


	        ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);   //La UART tiene que seguir funcionando aunque el micro este dormido
	        ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);   //La UART tiene que seguir funcionando aunque el micro este dormido


	        //Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	        RGBInit(1);
	        SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	        SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	        SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);  //Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo


	        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
	        // Configura el Timer0 para cuenta periodica de 16 bits con la mitad B del TIMER0
	        TimerConfigure(TIMER2_BASE,  TIMER_CFG_A_PERIODIC);
	        // Periodo de cuenta de mï¿½ximo posible, para medio TIMER (16bits)
	        uint32_t ui16Period;
	        ui16Period = SysCtlClockGet()/10;
	        // Carga la cuenta en el Timer0B
	        TimerLoadSet(TIMER2_BASE, TIMER_A, ui16Period);
	        // Habilita interrupcion del modulo TIMER
	        IntEnable(INT_TIMER2A);
	        // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta"
	       // TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	        // Habilita permiso general de interrupciones el sistema.
	        IntMasterEnable();
	        // Activa el Timer0B (empezara a funcionar)
	        IntPrioritySet(INT_TIMER2A,configMAX_SYSCALL_INTERRUPT_PRIORITY);
	       // TimerEnable(TIMERA_BASE, TIMER_B);

	      //  ACME_setPinDir(15);



	/********************************      Creacion de tareas *********************/

	//Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK,COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while(1);
    }

	//Esta funcion crea internamente una tarea para las comunicaciones USB.
	//Ademas, inicializa el USB y configura el perfil USB-CDC
	if (remotelink_init(REMOTELINK_TASK_STACK,REMOTELINK_TASK_PRIORITY,messageReceived)!=pdTRUE)
	{
	    while(1); //Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero remotelink.c
	}


	// Inicializa el ADC y crea una tarea...
	configADC_IniciaADC();
  /*  if((xTaskCreate(ADCTask, (portCHAR *)"ADC", 512,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }
    if((xTaskCreate(ADC1Task, (portCHAR *)"ADC1", 512,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
        {
            while(1);
        }*/

   if((xTaskCreate(EnvioTask,(portCHAR *)"Envio ",512,NULL,tskIDLE_PRIORITY +1,NULL)!= pdTRUE))
        {
            while(1);
        }

   /* Attempt to create the event group. */
       xCreatedEventGroup = xEventGroupCreate();

       /* Was the event group created successfully? */
       if( xCreatedEventGroup == NULL )
       {
           /* The event group was not created because there was insufficient
           FreeRTOS heap available. */
       }
       else
       {
           /* The event group was created. */
       }


        cola = xQueueCreate(5,sizeof(uint8_t));
         if (NULL == cola) while(1);

         g_pUARTSemaphore=xSemaphoreCreateMutex();
            if ((g_pUARTSemaphore==NULL))
            {
                while (1);  //No hay memoria para los semaforo
            }

	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

void RutinaISR(void){

    BaseType_t higherPriorityTaskWoken=pdFALSE;
    xEventGroupSetBitsFromISR(xCreatedEventGroup, BIT_0, &higherPriorityTaskWoken );
    uint8_t status=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
     xQueueSendFromISR(cola,&status,&higherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}




