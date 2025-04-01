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

bool endRun = false;


/*
#include <stdbool.h>
#define RED_LED 18  //PortB Pin 18
#define MASK(x) (1 << (x))
 
void Red(bool on){
	if(on)
		PTB->PCOR = MASK(RED_LED);
	else
		PTB->PSOR = MASK(RED_LED);
}
 void InitGPIO(void){
	// Enable Clock to PORTB
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK);
	
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);

	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= MASK(RED_LED);
}
*/
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
						//Red(1);
        }
        // else handle errors if needed
    }
}

// ---------------------------  tBrain ----------------------------

int isRunning=0;

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
        osSemaphoreAcquire(sem_uartRx, osWaitForever);

			if (!endRun){
			  // Dequeue all bytes that have arrived
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
						osThreadFlagsSet(play_buzzer_Id, 0x0001);
						
					}
					
						
						//movement_master_control(cmd);
            // Send the command to the Motor thread
            osMessageQueuePut(motorCommandQueue, &cmd, 0, 0);
        }
			}
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
		//InitGPIO();
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
    osThreadNew(tUART,  NULL, NULL); // The UART thread (which receives + decodes bytes)
    osThreadNew(tMotor, NULL, NULL); // The Motor thread (which applies the commands)
	
				
		osThreadNew(tBrain, NULL, NULL);
		osThreadNew(green_led_thread, NULL, NULL);
		osThreadNew(red_led_thread, NULL, NULL);

		play_buzzer_Id = osThreadNew(tAudio, NULL, NULL);
				
    // Start the RTOS scheduler
    osKernelStart();


		for (;;) {
		}

		/*
    for (;;) {
		
			// Test forward movement
			movement_master_control(forward);
			delay(2000000); // Turn Left 
			// Test backward movement
			movement_master_control(backward);
			delay(2000000); // Turn Right 

			// Test left turn
			movement_master_control(left);
			delay(2000000); // Turn Left 
			// Test right turn
			movement_master_control(right);
			delay(2000000); // Turn Right 
			// Stop the motors
			movement_master_control(idle);
			delay(2000000); 
			
		
		} // Should never reach here
		*/
}
