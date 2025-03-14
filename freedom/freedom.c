#include "MKL25Z4.h"                    // Device header

// #define UART2_RX_PIN 2
// #define UART2_TX_PIN 3
#define UART2_BAUD_RATE 9600
#define UART2_CLOCK 48000000

#define RED_LED_PIN 18   // PTB18
#define GREEN_LED_PIN 19 // PTB19
#define BLUE_LED_PIN 1   // PTD1

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22  // Page 162
#define UART_RX_PORTE23 23   // Page 162
#define UART2_INT_PRIO 128

// Global variable to store the forward/backward control level and left/right control level
// Structure to store movement control levels
struct movementControlMessage
{
    uint8_t forwardLevel;   // Level of forward movement (0-7)
    uint8_t backwardLevel;  // Level of backward movement (0-7)
    uint8_t leftLevel;      // Level of left movement (0-7)
    uint8_t rightLevel;     // Level of right movement (0-7)
};

// Create a queue 
#define Q_SIZE 128

typedef struct { 
    unsigned char DATA[Q_SIZE];
    unsigned int HEAD;
    unsigned int TAIL;
    unsigned int SIZE;
} Q_T;

Q_T tx_q, rx_q;

volatile uint8_t rx_IRQ_data = 0;

void Q_Init(Q_T *q) {
    unsigned int i;
    for(i=0; i < Q_SIZE; i++) q->DATA[i] = 0; // Initialise to 0
    q->HEAD = 0;
    q->TAIL = 0;
    q->SIZE = 0;
}

int Q_Empty(Q_T *q) {
    return q->SIZE == 0;
}

int Q_Full(Q_T *q) {
    return q->SIZE == Q_SIZE;
}

int Q_Enqueue(Q_T *q, unsigned char d) {
    if(Q_Full(q)) return 0; // Queue full - Failure
    q->DATA[q->TAIL++] = d;
    q->TAIL %= Q_SIZE; // This makes the list circular
    q->SIZE++;
    return 1; // Success
}

unsigned char Q_Dequeue(Q_T *q) {
    if(Q_Empty(q)) return 0; // Nothing to dequeue
    unsigned char t = q->DATA[q->HEAD];
    q->DATA[q->HEAD++] = 0;
    q->HEAD %= Q_SIZE;
    q->SIZE--;
    return t;
}

void UART2_IRQHandler(void) {
    // Transmit [executes when data is sent]
    NVIC_ClearPendingIRQ(UART2_IRQn);
    if(UART2->S1 & UART_S1_TDRE_MASK) {
        if(!Q_Empty(&tx_q)) {
            // Queue not empty so send data
            UART2->D = Q_Dequeue(&tx_q);
        } else {
            // Queue Empty so disable tx
            UART2->C2 &= ~UART_C2_TIE_MASK; 
        }
    }
    // Receive [executes when data is received]
    if(UART2->S1 & UART_S1_RDRF_MASK) {
        if(!Q_Full(&rx_q)) {
            Q_Enqueue(&rx_q, UART2->D);
        } else {
            // Queue Full - Error
            while(1); 
        }
    }
    // Error checking
    if(UART2->S1 & (UART_S1_OR_MASK | 
                    UART_S1_NF_MASK | 
                    UART_S1_FE_MASK | 
                    UART_S1_PF_MASK)) {
        // TODO: Handle error
        // TODO: Clear Flag
    }
}


void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    // Page 162 - PTE22 Alt 4 = UART2 TX
    // PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    // PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
    // Page 162 - PTE23 Alt 4 = UART2 RX
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
    // UART2->C2 - UART Control Register 2 (Page 753)
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); 
    // Default system clock - 48MHz
    bus_clock = (48000000) / 2;
    // 16 because clock is oversampled for noise control - unique to this controller
    divisor = bus_clock / (baud_rate * 16); 
    UART2->BDH = UART_BDH_SBR(divisor >> 8); // Baud High
    UART2->BDL = UART_BDL_SBR(divisor); // Baud Low

    // Page 751 - These are just to disable parity errors
    UART2->C1 = 0;  // Sets last bit PT (Parity) to 0 (no parity)
    UART2->S2 = 0;  // RAF Receiver Active Flag set to 0 - receiver waiting for start bit
    UART2->C3 = 0;  // Parity Error Interrupt set to 0 - disabled

    // Queue
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // Enable TX and RX Interrupts
    // UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
		UART2->C2 |= UART_C2_RIE_MASK;

    // Initialise 
		// Q_Init(&tx_q);
    Q_Init(&rx_q);

    // Enables Transmit & Receive
    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}

/* UART2 Transmit Poll */
void UART2_Transmit_Poll(uint8_t data) {
    // S1 = Status Register 1, TDRE = Transmit Data Register Empty
    while(!(UART2->S1 & UART_S1_TDRE_MASK)); // Poll until Data register is empty
    UART2->D = data; // put into D register
    // in transmit mode , waiting for data to be set
}

uint8_t UART2_Receive_Poll(void) {
    // S1 = Status Register 1, RDRF = Receive Data Register Full Flag
    while(!(UART2->S1 & UART_S1_RDRF_MASK)); // Poll until Full
    return (UART2->D); // Return data
}

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}


void LED_Init(void) {
    // Enable clock for Port B and Port D
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    // Configure LED pins as GPIO
    PORTB->PCR[RED_LED_PIN] = PORT_PCR_MUX(1);
    PORTB->PCR[GREEN_LED_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[BLUE_LED_PIN] = PORT_PCR_MUX(1);

    // Set LED pins as outputs
    PTB->PDDR |= (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN);
    PTD->PDDR |= (1 << BLUE_LED_PIN);

    // Turn off LEDs initially
    PTB->PSOR = (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN);
    PTD->PSOR = (1 << BLUE_LED_PIN);
}

void set_LED_intensity(uint8_t red, uint8_t green, uint8_t blue) {
    // Set the intensity of the LEDs based on the control levels
    if (red > 0) {
        PTB->PCOR = (1 << RED_LED_PIN); // Turn on RED LED
    } else {
        PTB->PSOR = (1 << RED_LED_PIN); // Turn off RED LED
    }

    if (green > 0) {
        PTB->PCOR = (1 << GREEN_LED_PIN); // Turn on GREEN LED
    } else {
        PTB->PSOR = (1 << GREEN_LED_PIN); // Turn off GREEN LED
    }

    if (blue > 0) {
        PTD->PCOR = (1 << BLUE_LED_PIN); // Turn on BLUE LED
    } else {
        PTD->PSOR = (1 << BLUE_LED_PIN); // Turn off BLUE LED
    }
}

void decode_packet(volatile uint8_t *packet, uint8_t length) {
    if (length < 1) {
        // Packet too short to contain data
      set_LED_intensity(4, 4, 4);  
			return;
    }

    uint8_t data = packet[0];

    // Extract control levels from the received byte
    uint8_t leftLevel = (data >> 0) & 0x03; 		// 11
    uint8_t rightLevel = (data >> 2) & 0x03; 		// 11
    uint8_t forwardLevel = (data >> 4) & 0x03; 	// 11
    uint8_t backwardLevel = (data >> 6) & 0x03; // 11

    // Set LED intensities based on control levels
    
		if (backwardLevel) {
			set_LED_intensity(1,1,0);
		}
		else{
			set_LED_intensity(forwardLevel, leftLevel, rightLevel);
		}
    // Process the extracted control levels
    // Implement your control logic here
}

// Function to decode the received byte and map it to motor duty cycles
void decode_motor_control(uint8_t data) {
    // Extract the upper 4 bits (forward/backward control)
    uint8_t forwardBackward = (data >> 4) & 0x0F;

    // Extract the lower 4 bits (left/right control)
    uint8_t leftRight = data & 0x0F;

    // Create a structure to store the movement control levels
    struct movementControlMessage controlMessage;

    // Populate the forward/backward fields
    if (forwardBackward <= 7) {
        // Values 0000–0111 indicate backward movement
        controlMessage.forwardLevel = 0; // No forward movement
        controlMessage.backwardLevel = 7 - forwardBackward; // Map 0000 to 7, 0111 to 0
    } else if (forwardBackward >= 8 && forwardBackward <= 14) {
        // Values 1000–1110 indicate forward movement
        controlMessage.forwardLevel = forwardBackward - 8; // Map 1000 to 1, 1110 to 7
        controlMessage.backwardLevel = 0; // No backward movement
    } else {
        // Value 1111 is unused
        controlMessage.forwardLevel = 0;
        controlMessage.backwardLevel = 0;
    }

    // Populate the left/right fields based on the mapping
    if (leftRight <= 7) {
        // Values 0000–0111 indicate left movement
        controlMessage.leftLevel = 7 - leftRight; // Map 0000 to 7, 0111 to 0
        controlMessage.rightLevel = 0; // No right movement
    } else if (leftRight >= 8 && leftRight <= 14) {
        // Values 1000–1110 indicate right movement
        controlMessage.rightLevel = leftRight - 8; // Map 1000 to 1, 1110 to 7
        controlMessage.leftLevel = 0; // No left movement
    } else {
        // Value 1111 is unused
        controlMessage.leftLevel = 0;
        controlMessage.rightLevel = 0;
    }

    // call method to set motor duty cycles
    (controlMessage);

}

int main(void) {
    SystemCoreClockUpdate();
    initUART2(BAUD_RATE);
    LED_Init();
	
		// set_LED_intensity(4,0,0);

    while (1) {
        if (!Q_Empty(&rx_q)) {
            uint8_t data = Q_Dequeue(&rx_q);
            decode_packet(&data, 1);
        }
    }
}
