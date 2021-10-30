/*
 * remotelink_messages.h
 *
 *  Created on: March. 2021
 *
 */

#ifndef RL_MESSAGES_H
#define RL_MESSAGES_H
//Codigos de los mensajes y definicion de parametros para el protocolo RPC

// IMPORTANTE el orden de los comandos debe SER EL MISMO aqui, y en el codigo equivalente en la parte de QT

typedef enum {
    MESSAGE_REJECTED,
    MESSAGE_PING,
    MESSAGE_LED_GPIO,
    MESSAGE_LED_PWM_BRIGHTNESS,
    MESSAGE_ADC_SAMPLE,
    MESSAGE_MODO,
    MESSAGE_COLOR,
    MESSAGE_SONDEO,
    MESSAGE_INTERRUPCION,
    MESSAGE_TIMER,
    MESSAGE_ACTIVA_TIMER,
    MESSAGE_PROMEDIO_HARDWARE,
    MESSAGE_FRECUENCIA,
    MESSAGE_ACME,


    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad en ambos extremos

#pragma pack(1) //Cambia el alineamiento de datos en memoria a 1 byte.

typedef struct {
    uint8_t command;
} MESSAGE_REJECTED_PARAMETER;

typedef union{
    struct {
         uint8_t padding:1;
         uint8_t red:1;
         uint8_t blue:1;
         uint8_t green:1;
    } leds;
    uint8_t value;
} MESSAGE_LED_GPIO_PARAMETER;

typedef struct {
    uint8_t gpio0;
    uint8_t gpio1;
    uint8_t gpio2;
    uint8_t gpio3;
} MESSAGE_ACME_PARAMETER;

typedef struct {
    int x;
} MESSAGE_MODO_PARAMETER;

typedef struct {
    int r;
    int g;
    int b;
} MESSAGE_COLOR_PARAMETER;

typedef struct {
    uint8_t led_1;
    uint8_t led_2;
} MESSAGE_SONDEO_PARAMETER;

typedef struct {
    int Int;

} MESSAGE_INTERRUPCION_PARAMETER;

typedef struct {
    float rIntensity;
} MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER;

typedef struct {
    float frec;
} MESSAGE_FRECUENCIA_PARAMETER;

typedef struct {
    int Phar;
} MESSAGE_PROMEDIO_HARDWARE_PARAMETER;

typedef struct {
    int Tim;

} MESSAGE_ACTIVA_TIMER_PARAMETER;

typedef struct{
    uint16_t chan1[5];
    uint16_t chan2[5];
    uint16_t chan3[5];
    uint16_t chan4[5];
    uint16_t chan5[5];
    uint16_t chan6[5];
} MESSAGE_TIMER_PARAMETER;

typedef struct
{
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
} MESSAGE_ADC_SAMPLE_PARAMETER;

#pragma pack()  //...Pero solo para los comandos que voy a intercambiar, no para el resto.

#endif // RPCCOMMANDS_H

