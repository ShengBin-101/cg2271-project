// main.c
// Entry point: initializes peripherals, creates RTOS objects and threads, then starts the scheduler.

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#include "motor.h"
#include "uart.h"
#include "led.h"
#include "buzzer.h"

// RTOS handles
osMessageQueueId_t motorCommandQueue;
osMessageQueueId_t red_led_message_queue, green_led_message_queue;
osThreadId_t       play_buzzer_Id;

// Shared flags
volatile bool endRun = false;  // Signals when “finish” command received
int           isRunning = 0;   // Indicates robot is moving

// Simple busy-wait delay (just for tiny timing tweaks)
static void delay(volatile uint32_t count) {
    while (count--) {
        __asm("NOP");
    }
}

// Motor thread: waits for new motor commands and applies them
void tMotor(void *arg) {
    struct movementControlMessage cmd;
    for (;;) {
        if (osMessageQueueGet(motorCommandQueue, &cmd, NULL, osWaitForever) == osOK) {
            movement_master_control(cmd);
        }
    }
}

// Brain thread: updates LED threads with current running state
void tBrain(void *arg) {
    for (;;) {
        osMessageQueuePut(red_led_message_queue, &isRunning, 0, 0);
        osMessageQueuePut(green_led_message_queue, &isRunning, 0, 0);
        osDelay(10);  // small pause to avoid flooding
    }
}

// UART thread: waits for UART data, decodes into motor commands, signals buzzer
void tUART(void *arg) {
    for (;;) {
        if (!endRun) {
            // Block until at least one byte arrives
            osSemaphoreAcquire(sem_uartRx, osWaitForever);
            // Notify buzzer that we have fresh data
            osThreadFlagsSet(play_buzzer_Id, 0x0001);

            // Drain the receive queue
            while (!Q_Empty(&rx_q)) {
                uint8_t data = Q_Dequeue(&rx_q);
                struct movementControlMessage cmd = decode_motor_control(data);

                // Determine running state
                isRunning = (cmd.forwardLevel || cmd.backwardLevel || cmd.leftLevel || cmd.rightLevel);

                if (cmd.finish) {
                    // Received “finish” command: stop motors, flag end of run
                    endRun = true;
                    movement_master_control(idle);
                } else {
                    // Send to motor thread
                    osMessageQueuePut(motorCommandQueue, &cmd, 0, 0);
                }
            }
        } else {
            // Once finished, play alternate buzzer tune
            osThreadFlagsSet(play_buzzer_Id, 0x0002);
        }
    }
}

// UART2 interrupt: enqueues received bytes, handles errors, stops when endRun==true
void UART2_IRQHandler(void) {
    if (endRun) {
        UART2->C2 &= ~UART_C2_RIE_MASK;  // disable further RX interrupts
        return;
    }
    NVIC_ClearPendingIRQ(UART2_IRQn);

    // Data received?
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        if (!Q_Full(&rx_q)) {
            Q_Enqueue(&rx_q, UART2->D);
            osSemaphoreRelease(sem_uartRx);
        } else {
            for (;;); // RX queue overflow—halt here (or handle error)
        }
    }

    // Clear any framing/parity errors
    if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
        (void)UART2->D;
    }
}

int main(void) {
    // 1) Initialize system clock and peripherals
    SystemCoreClockUpdate();
    initLED();
    initMotorPWM();
    initUART2(BAUD_RATE);
    initBuzzerPWM();

    // 2) Initialize RTOS
    osKernelInitialize();

    // 3) Create message queues
    red_led_message_queue   = osMessageQueueNew(1, sizeof(int), NULL);
    green_led_message_queue = osMessageQueueNew(1, sizeof(int), NULL);
    motorCommandQueue       = osMessageQueueNew(8, sizeof(struct movementControlMessage), NULL);

    // 4) Create threads
    osThreadNew(tUART,            NULL, NULL);
    osThreadNew(tMotor,           NULL, NULL);
    osThreadNew(tBrain,           NULL, NULL);
    osThreadNew(green_led_thread, NULL, NULL);
    osThreadNew(red_led_thread,   NULL, NULL);
    play_buzzer_Id = osThreadNew(tAudio, NULL, NULL);

    // 5) Start scheduler
    osKernelStart();
    for (;;) { }  // should never reach here
}
