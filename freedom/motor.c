/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"      

#define MIN_SPEED 700
#define MAX_SPEED 20000
 
 struct movementControlMessage
{
    uint8_t forwardLevel;   // Level of forward movement (0-7)
    uint8_t backwardLevel;  // Level of backward movement (0-7)
    uint8_t leftLevel;      // Level of left movement (0-7)
    uint8_t rightLevel;     // Level of right movement (0-7)
};

// Motor pin assignments
#define LEFT_FORWARD 1 		// 	TPM0_C0 - PD0: Connects to left motor driver (A1 & B1)
#define LEFT_BACKWARD 0 	//	TPM0_C1 - PD1: Connects to left motor driver (A2 & B2)

#define RIGHT_FORWARD 3 	//	TPM0_C2 - PD2: Connects to right motor driver (A1 & B1)
#define RIGHT_BACKWARD 2 	//	TPM0_C3 - PD3: Connects to right motor driver (A2 & B2)

// TPM channel mappings
#define LEFT_FORWARD_CV   TPM0_C0V
#define LEFT_BACKWARD_CV  TPM0_C1V

#define RIGHT_FORWARD_CV    TPM0_C2V
#define RIGHT_BACKWARD_CV   TPM0_C3V

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}


void initMotorPWM(void) {
    //init 2 pins, 1 for forward, 1 for backward
    //__disable_irq();
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    //left
    PORTD->PCR[LEFT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[LEFT_BACKWARD] |= PORT_PCR_MUX(4);
    PORTD->PCR[LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[LEFT_FORWARD] |= PORT_PCR_MUX(4);
 
    //right
    PORTD->PCR[RIGHT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[RIGHT_BACKWARD] |= PORT_PCR_MUX(4);
    PORTD->PCR[RIGHT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[RIGHT_FORWARD] |= PORT_PCR_MUX(4);

    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
 
    TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		
		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	  
    TPM0->MOD = 7,500;
    
	  TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	  TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
    
    TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    
    TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 
    TPM0_C0V = 0; // LEFT_FORWARD_CV
    TPM0_C1V = 0; // LEFT_BACKWARD_CV
    TPM0_C2V = 0; // RIGHT_FORWARD_CV
    TPM0_C3V = 0; // RIGHT_BACKWARD_CV

}

// Helper function to scale speed non-linearly
/*
int scale_speed(float normalizedSpeed) {
  float absSpeed = fabs(normalizedSpeed);
  int scaledSpeed;

  if (absSpeed <= 0.5) {
      // For lower levels (0.0 to 0.5), use slower scaling
      scaledSpeed = (int)(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (absSpeed * absSpeed)); // Quadratic scaling
  } else {
      // For higher levels (0.5 to 1.0), use faster scaling
      scaledSpeed = (int)(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * absSpeed);
  }

  return scaledSpeed;
}*/

void movement_master_control(struct movementControlMessage msg) {
  // Define a neutral zone threshold
  const float NEUTRAL_THRESHOLD = 0.1;

  // Normalize input levels to range -1.0 to 1.0
  float linearSpeed = (msg.forwardLevel - msg.backwardLevel) / 7.0;   // Forward/backward motion     
  float angularSpeed = (msg.leftLevel - msg.rightLevel) / 7.0;        // Left/right turning

  // Apply the neutral zone
//  if (fabs(linearSpeed) < NEUTRAL_THRESHOLD) {
//      linearSpeed = 0.0;
//  }
 // if (fabs(angularSpeed) < NEUTRAL_THRESHOLD) {
//      angularSpeed = 0.0;
//  }

  // Calculate normalized wheel speeds
  float leftWheelSpeedNormalized = linearSpeed - angularSpeed;
  float rightWheelSpeedNormalized = linearSpeed + angularSpeed;

  // Scale normalized speeds to motor PWM range with non-linear scaling
   int leftWheelSpeed = leftWheelSpeedNormalized;//scale_speed(leftWheelSpeedNormalized);
   int rightWheelSpeed = rightWheelSpeedNormalized;//scale_speed(rightWheelSpeedNormalized);
  
  // Set motor directions and speeds
  if (leftWheelSpeedNormalized > 0) {
      // Left wheel forward
      // LEFT_FORWARD_CV = leftWheelSpeed;
      // LEFT_BACKWARD_CV = 0;
      LEFT_FORWARD_CV = 3499; // 50% duty cycle
      LEFT_BACKWARD_CV = 0;
  } else if (leftWheelSpeedNormalized < 0) {
      // Left wheel backward
      // LEFT_FORWARD_CV = 0;
      // LEFT_BACKWARD_CV = leftWheelSpeed;
      LEFT_FORWARD_CV = 0;
      LEFT_BACKWARD_CV = 3499; // 50% duty cycle
  } else {
      // Left wheel stop
      LEFT_FORWARD_CV = 0;
      LEFT_BACKWARD_CV = 0;
  }

  if (rightWheelSpeedNormalized > 0) {
      // Right wheel forward
      // RIGHT_FORWARD_CV = rightWheelSpeed;
      // RIGHT_BACKWARD_CV = 0;
      RIGHT_FORWARD_CV = 3499; // 50% duty cycle
      RIGHT_BACKWARD_CV = 0;
  } else if (rightWheelSpeedNormalized < 0) {
      // Right wheel backward
      // RIGHT_FORWARD_CV = 0;
      // RIGHT_BACKWARD_CV = rightWheelSpeed;
      RIGHT_FORWARD_CV = 0;
      RIGHT_BACKWARD_CV = 3499; // 50% duty cycle
  } else {
      // Right wheel stop
      RIGHT_FORWARD_CV = 0;
      RIGHT_BACKWARD_CV = 0;
  }
}

struct movementControlMessage idle = {0, 0, 0, 0};
  struct movementControlMessage forward = {7, 0, 0, 0};
  struct movementControlMessage backward = {0, 7, 0, 0};
  struct movementControlMessage left = {0, 0, 7, 0};
  struct movementControlMessage right = {0, 0, 0, 7};

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 

  
  // ...
  for (;;) {
    // Sample control message
  }
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  initMotorPWM();
  // ...
 
  //osKernelInitialize();                 // Initialize CMSIS-RTOS
  //osThreadNew(app_main, NULL, NULL);    // Create application main thread
  //osKernelStart();                      // Start thread execution
  for (;;) {
	  

  // Test forward movement
  movement_master_control(forward);
  delay(20000000); // Turn Left 
  // Test backward movement
  movement_master_control(backward);
	delay(20000000); // Turn Right 

  // Test left turn
  movement_master_control(left);
	delay(20000000); // Turn Left 
  // Test right turn
  movement_master_control(right);
  delay(20000000); // Turn Right 
  // Stop the motors
  movement_master_control(idle);
	delay(20000000); 
	}
}
