#include "MKL25Z4.h"                    // Device header

// UART pin assignments
// #define UART2_RX_PIN 2
// #define UART2_TX_PIN 3
#define UART2_BAUD_RATE 9600
#define UART2_CLOCK 48000000

//LED pin assignments
#define RED_LED_PIN 18   // PTB18
#define GREEN_LED_PIN 19 // PTB19
#define BLUE_LED_PIN 1   // PTD1

// Motor pin assignments
#define FRONT_RIGHT_FORWARD 1 //PB1, TPM1C1, 
#define FRONT_RIGHT_BACKWARD 0 //PB0, TPM1C0

#define REAR_RIGHT_FORWARD 3 //PB3, TPM2C1, 
#define REAR_RIGHT_BACKWARD 2 //PB2, TPM2C0

#define FRONT_LEFT_FORWARD 31 //PE31, TPM0C4, 
#define FRONT_LEFT_BACKWARD 3 //PD3, TPM0C3

#define REAR_LEFT_FORWARD 2 //PD2, TPM0C2, 
#define REAR_LEFT_BACKWARD 0 //PD0, TPM0C0

// TPM channel mappings
#define FRONT_RIGHT_FORWARD_CV   TPM1_C1V
#define FRONT_RIGHT_BACKWARD_CV  TPM1_C0V

#define REAR_RIGHT_FORWARD_CV    TPM2_C1V
#define REAR_RIGHT_BACKWARD_CV   TPM2_C0V

#define FRONT_LEFT_FORWARD_CV		TPM0_C4V
#define FRONT_LEFT_BACKWARD_CV 	TPM0_C3V

#define REAR_LEFT_FORWARD_CV   TPM0_C2V
#define REAR_LEFT_BACKWARD_CV  TPM0_C0V

//UART constants
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

/** MOTOR CONTROL FUNCTIONS **/
void initMotorPWM(void) {
    //init 2 pins, 1 for forward, 1 for backward
    //__disable_irq();
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
 
    //front left
    PORTD->PCR[FRONT_LEFT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[FRONT_LEFT_BACKWARD] |= PORT_PCR_MUX(4);
	  PORTE->PCR[FRONT_LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[FRONT_LEFT_FORWARD] |= PORT_PCR_MUX(3);
	 
	  //rear left 
	  PORTD->PCR[REAR_LEFT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[REAR_LEFT_BACKWARD] |= PORT_PCR_MUX(4);
	  PORTD->PCR[REAR_LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[REAR_LEFT_FORWARD] |= PORT_PCR_MUX(4);
	 
	  //front right
	  PORTB->PCR[FRONT_RIGHT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[FRONT_RIGHT_BACKWARD] |= PORT_PCR_MUX(3);
	  PORTB->PCR[FRONT_RIGHT_FORWARD] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[FRONT_RIGHT_FORWARD] |= PORT_PCR_MUX(3);
	 
	  //rear right 
	  PORTB->PCR[REAR_RIGHT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[REAR_RIGHT_BACKWARD] |= PORT_PCR_MUX(3);
	  PORTB->PCR[REAR_RIGHT_FORWARD] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[REAR_RIGHT_FORWARD] |= PORT_PCR_MUX(3);
	 
	  SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	  SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	  SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	 
	  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	 
	  //TPM0 configuration
	  TPM0->MOD = 18750000;
	 
	  TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	  TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	  TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	 
	  TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	  TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
	  TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	  TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
 	  TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
	  //TPM1 configuration
	  TPM1->MOD = 18750000;
	 
	  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	  TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	 
	  TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	  TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
	  //TPM2 configuration
	  TPM2->MOD = 18750000;
	 
	  TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	  TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	  TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	 
	  TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	  TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	  TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
	 //__enable_irq();
}

void move_forward(uint8_t forward_level){
//		int speed = 1400 * (int) forward_level;
	  int speed = (forward_level == 0) ? 0 : (int) (1400 + (800 + forward_level * 100) * (forward_level - 1));
	
    FRONT_RIGHT_BACKWARD_CV = 0;
    FRONT_RIGHT_FORWARD_CV = speed;
	
    REAR_RIGHT_BACKWARD_CV = 0;
    REAR_RIGHT_FORWARD_CV = speed;
	
    FRONT_LEFT_BACKWARD_CV = 0;
    FRONT_LEFT_FORWARD_CV = speed;
	
    REAR_LEFT_BACKWARD_CV = 0;
    REAR_LEFT_FORWARD_CV = speed;
}

void move_backward(uint8_t backward_level){
//		int speed = 1400 * (int) backward_level;
	  int speed = (backward_level == 0) ? 0 : (int) (1400 + (800 + backward_level * 100) * (backward_level - 1));
    
		FRONT_RIGHT_BACKWARD_CV = speed;
    FRONT_RIGHT_FORWARD_CV = 0;
	
    REAR_RIGHT_BACKWARD_CV = speed;
    REAR_RIGHT_FORWARD_CV = 0;
	
    FRONT_LEFT_BACKWARD_CV = speed;
    FRONT_LEFT_FORWARD_CV = 0;
	
    REAR_LEFT_BACKWARD_CV = speed;
    REAR_LEFT_FORWARD_CV = 0;
}

void move_right() {
    FRONT_RIGHT_BACKWARD_CV = 0;
    FRONT_RIGHT_FORWARD_CV = 4000;
	
    REAR_RIGHT_BACKWARD_CV = 0;
    REAR_RIGHT_FORWARD_CV = 0;
	
    FRONT_LEFT_BACKWARD_CV = 0;
    FRONT_LEFT_FORWARD_CV = 12000;
	
    REAR_LEFT_BACKWARD_CV = 0;
    REAR_LEFT_FORWARD_CV = 12000;
}

void move_left() {
    FRONT_RIGHT_BACKWARD_CV = 0;
    FRONT_RIGHT_FORWARD_CV = 47000;
	
    REAR_RIGHT_BACKWARD_CV = 0;
    REAR_RIGHT_FORWARD_CV = 47000;
	
    FRONT_LEFT_BACKWARD_CV = 0;
    FRONT_LEFT_FORWARD_CV = 0;
	
    REAR_LEFT_BACKWARD_CV = 0;
    REAR_LEFT_FORWARD_CV = 2000;
}

void move_left_on_spot(uint8_t left_level) {
	  int speed = (left_level == 0) ? 0 : (1700 + 700 * (int) (left_level - 1));
    FRONT_RIGHT_BACKWARD_CV = 0;
    FRONT_RIGHT_FORWARD_CV = speed;
	
    REAR_RIGHT_BACKWARD_CV = 0;
    REAR_RIGHT_FORWARD_CV = speed;
	
    FRONT_LEFT_BACKWARD_CV = speed;
    FRONT_LEFT_FORWARD_CV = 0;
	
    REAR_LEFT_BACKWARD_CV = speed;
    REAR_LEFT_FORWARD_CV = 0;
}

void move_right_on_spot(uint8_t right_level) {
	  int speed = (right_level == 0) ? 0 : (1700 + 700 * (int) (right_level - 1));

    FRONT_RIGHT_BACKWARD_CV = speed;
    FRONT_RIGHT_FORWARD_CV = 0;
	
    REAR_RIGHT_BACKWARD_CV = speed;
    REAR_RIGHT_FORWARD_CV = 0;
	
    FRONT_LEFT_BACKWARD_CV = 0;
    FRONT_LEFT_FORWARD_CV = speed;
	
    REAR_LEFT_BACKWARD_CV = 0;
    REAR_LEFT_FORWARD_CV = speed;
}

void stop_movement() {
    FRONT_RIGHT_BACKWARD_CV = 0;
    FRONT_RIGHT_FORWARD_CV = 0;
	
    REAR_RIGHT_BACKWARD_CV = 0;
    REAR_RIGHT_FORWARD_CV = 0;
	
    FRONT_LEFT_BACKWARD_CV = 0;
    FRONT_LEFT_FORWARD_CV = 0;
	
    REAR_LEFT_BACKWARD_CV = 0;
    REAR_LEFT_FORWARD_CV = 0;
}

void movement_master_control(struct movementControlMessage msg) {
		uint8_t f = msg.forwardLevel;
		uint8_t b = msg.backwardLevel;
		uint8_t l = msg.leftLevel;
		uint8_t r = msg.rightLevel;
	
		if (f > 0 && b == 0 && l == 0 && r == 0) {
				move_forward(f);
		} else if (b > 0 && f == 0 && l == 0 && r == 0) {
				move_backward(b);
		} else if (l > 0 && f == 0 && b == 0 && r == 0){
				move_left_on_spot(l);
		}	else if (r > 0 && f == 0 && b == 0 && l == 0) {
				move_right_on_spot(r);
		}	else if (l > 0 && f > 0 && b == 0 && r == 0) {
				move_left();
		}	else if (r > 0 && f > 0 && b == 0 && l == 0) {
				move_right();
		}	else {
				stop_movement();
		}
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
		movement_master_control(controlMessage);
}

int main(void) {
    SystemCoreClockUpdate();
    initUART2(BAUD_RATE);
		initMotorPWM();

    LED_Init();	

    while (1) {
        if (!Q_Empty(&rx_q)) {
            uint8_t data = Q_Dequeue(&rx_q);
            decode_packet(&data, 1);
        }
    }
}
