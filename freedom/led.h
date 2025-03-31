#ifndef LED_H
#define LED_H

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_PIN1 1 //PTA1
#define RED_PIN2 2 //PTA2
#define RED_PIN3 4 //PTD4
#define RED_PIN4 12 //PTA12
#define RED_PIN5 4 //PTA4
#define RED_PIN6 5 //PTA5 
#define RED_PIN7 8 //PTC8
#define RED_PIN8 9 //PTC9

#define GREEN_PIN1 7 //PTC7
#define GREEN_PIN2 0 //PTC0
#define GREEN_PIN3 3 //PTC3
#define GREEN_PIN4 4 //PTC4
#define GREEN_PIN5 5 //PTC5
#define GREEN_PIN6 6 //PTC6
#define GREEN_PIN7 10 //PTC10
#define GREEN_PIN8 11 //PTC11

#define MASK(x) (1 << (x))

extern osMessageQueueId_t red_led_message_queue, green_led_message_queue;

void initLED(void);
void red_led_thread(void *argument);
void green_led_thread(void *argument);
void control_thread(void *argument);

#endif // LED_H
