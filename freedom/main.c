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

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#include "motor.h"   // for initMotorPWM(), decode_motor_control(), movement_master_control()
#include "uart.h"    // for initUART2(), rx_q, Q_Empty(), Q_Dequeue()
#include "led.h"
#include "buzzer.h"

// A message queue handle for motor commands
osMessageQueueId_t motorCommandQueue;
osMessageQueueId_t red_led_message_queue, green_led_message_queue;

osThreadId_t play_buzzer_Id;

volatile bool endRun = false;

// --------------------------- Motor Thread ---------------------------
// This thread waits for new motor commands and applies them

void tMotor(void *argument) {
    struct movementControlMessage cmd;
    for (;;) {
        // Block until a new command arrives
        osStatus_t status = osMessageQueueGet(motorCommandQueue, &cmd, NULL, osWaitForever);
        if (status == osOK) {
            // Apply the new motor command
            movement_master_control(cmd);
        }
        // else handle errors if needed
    }
}

// ---------------------------  tBrain ----------------------------

int isRunning = 0;

void tBrain(void *argument) {
	
	for (;;){
		osMessageQueuePut(red_led_message_queue, &isRunning, NULL, 0);
		osMessageQueuePut(green_led_message_queue, &isRunning, NULL, 0);
	}
}

// --------------------------- UART Thread ----------------------------
// This thread is signaled by the UART semaphore whenever new bytes arrive.
// It decodes those bytes into motor commands and posts them to the motor queue.

void tUART(void *argument) {
    for (;;) {
        // Block until at least one byte arrives


		if (!endRun){
      osSemaphoreAcquire(sem_uartRx, osWaitForever);
			// Dequeue all bytes that have arrived
			osThreadFlagsSet(play_buzzer_Id, 0x0001);
			while (!Q_Empty(&rx_q)) {
					uint8_t data = Q_Dequeue(&rx_q);

					// Convert the single byte into a motor command
					struct movementControlMessage cmd = decode_motor_control(data);
					if (cmd.forwardLevel>0 || cmd.backwardLevel>0 || cmd.leftLevel>0 || cmd.rightLevel>0){
						isRunning = 1;
						}
					else {
						isRunning = 0;
					}
					if (cmd.finish) {
						endRun = true;	
						movement_master_control(idle);		// LAST ADDED LINE
					}
					else {
						// Send the command to the Motor thread
						osMessageQueuePut(motorCommandQueue, &cmd, 0, 0);

					}
				}
			}
			else{
				osThreadFlagsSet(play_buzzer_Id, 0x0002);
			}
    }
}				

void UART2_IRQHandler(void) {
    // Check if endRun is true
    if (endRun) {
        // Disable UART RX interrupt
        UART2->C2 &= ~UART_C2_RIE_MASK;
        return; // Exit the interrupt handler
    }

    NVIC_ClearPendingIRQ(UART2_IRQn); // Clear pending interrupt

    // RX interrupt - data received
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        if (!Q_Full(&rx_q)) {
            Q_Enqueue(&rx_q, UART2->D); // Read received byte into queue
            osSemaphoreRelease(sem_uartRx); // Signal semaphore for new data
        } else {
            while (1); // RX queue overflow - halt (can be replaced with error handling)
        }
    }

    // Handle UART errors: overrun, noise, framing, or parity errors
    if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | 
                     UART_S1_FE_MASK | UART_S1_PF_MASK)) {
        // Just clear the flags by reading S1 and D (data)
        (void)UART2->D;
    }
}


static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

int main(void) {
    // System + Peripheral Init
    SystemCoreClockUpdate();
		initLED();
    initMotorPWM();
    initUART2(BAUD_RATE);
		initBuzzerPWM();
		
    // RTOS Init
    osKernelInitialize();

    // Create the motor command queue
    // Each message = one movementControlMessage
  
		red_led_message_queue = osMessageQueueNew(1, sizeof(int), NULL);
		green_led_message_queue = osMessageQueueNew(1, sizeof(int), NULL);  
	
		motorCommandQueue = osMessageQueueNew(
        8, // capacity: up to 8 messages in the queue
        sizeof(struct movementControlMessage),
        NULL
    );
	
    // Create threads
  osThreadNew(tUART,  NULL, NULL); 	// The UART thread (which receives + decodes bytes)
  osThreadNew(tMotor, NULL, NULL); 	// The Motor thread (which applies the commands)
	osThreadNew(tBrain, NULL, NULL);	// The Brain thread (which determines the state of the robot)
	osThreadNew(green_led_thread, NULL, NULL);
	osThreadNew(red_led_thread, NULL, NULL);

	play_buzzer_Id = osThreadNew(tAudio, NULL, NULL);

	
    // Start the RTOS scheduler
    osKernelStart();
	// We should never get here as control is now taken by the scheduler
	for (;;) {}
}
