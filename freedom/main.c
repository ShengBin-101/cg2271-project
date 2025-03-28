/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "motor.h"
#include "led.h"
#include "buzzer.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
	
	// UART_thread
	/*
		Hardware Interrupt upon message receive, 
		Decode messages
		updates messageQueues & Flags Accordingly
	*/

	/* tBrain 
		Purpose is to determine Robot State
		Should take in messageQueue (type undecided yet) from UART_thread

		Updates messageQueue for motor_thread, to move motors
		
		Update messageQueue led_threads, to change/toggle LED frequency

		Updates thread flag for buzzer_thread
			To signal end of run, use some button press on controller. UART message currently has 1 bit unused, 
			could use that for button press
	
	*/
	
	// motor_thread - messageQueue containing control messages (struct)
	/*
		Takes retrieved struct and move motors
	*/
	
	// led_threads - messagequeue 
	/* 
		osMessageQueuePut(red_led_message_queue, &isRunning, NULL, 0);
		osMessageQueuePut(green_led_message_queue, &isRunning, NULL, 0);
	*/
	
	// buzzer_thread - Thread Flag
	/*

	
		when flag is set, play_tune 
	*/
	
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
