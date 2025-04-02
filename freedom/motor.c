#include "MKL25Z4.h"
#include "motor.h"
#include <math.h> 

struct movementControlMessage idle = {0, 0, 0, 0};
struct movementControlMessage forward = {7, 0, 0, 0};
struct movementControlMessage backward = {0, 7, 0, 0};
struct movementControlMessage left = {0, 0, 7, 0};
struct movementControlMessage right = {0, 0, 0, 7};

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
	  
    TPM0->MOD = 7500;
    
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

void movement_master_control(struct movementControlMessage msg) {
    // Define a neutral zone threshold
    const float NEUTRAL_THRESHOLD = 0.1;

    // Define a scaling factor for steering sensitivity (lower = less sensitive)
    const float STEERING_SENSITIVITY = 0.5; // Adjust this value (e.g., 0.5 = 50% sensitivity)

    // Normalize input levels to range -1.0 to 1.0
    float linearSpeed = (msg.forwardLevel - msg.backwardLevel) / 7.0;   // Forward/backward motion     
    float angularSpeed = (msg.leftLevel - msg.rightLevel) / 7.0;        // Left/right turning

    // Apply the neutral zone
    if (fabs(linearSpeed) < NEUTRAL_THRESHOLD) {
        linearSpeed = 0.0;
    }
    if (fabs(angularSpeed) < NEUTRAL_THRESHOLD) {
        angularSpeed = 0.0;
    }

    // Reduce steering sensitivity
    angularSpeed *= STEERING_SENSITIVITY;

    // Calculate normalized wheel speeds
    float leftWheelSpeedNormalized = linearSpeed - angularSpeed;
    float rightWheelSpeedNormalized = linearSpeed + angularSpeed;

    // Scale normalized speeds to motor PWM range
    int leftWheelSpeed = leftWheelSpeedNormalized * MAX_SPEED;
    int rightWheelSpeed = rightWheelSpeedNormalized * MAX_SPEED;

    // Set motor directions and speeds
    if (leftWheelSpeedNormalized > 0) {
        // Left wheel forward
        LEFT_FORWARD_CV = leftWheelSpeed;
        LEFT_BACKWARD_CV = 0;
    } else if (leftWheelSpeedNormalized < 0) {
        // Left wheel backward
        LEFT_FORWARD_CV = 0;
        LEFT_BACKWARD_CV = -leftWheelSpeed;
    } else {
        // Left wheel stop
        LEFT_FORWARD_CV = 0;
        LEFT_BACKWARD_CV = 0;
    }

    if (rightWheelSpeedNormalized > 0) {
        // Right wheel forward
        RIGHT_FORWARD_CV = rightWheelSpeed;
        RIGHT_BACKWARD_CV = 0;
    } else if (rightWheelSpeedNormalized < 0) {
        // Right wheel backward
        RIGHT_FORWARD_CV = 0;
        RIGHT_BACKWARD_CV = -rightWheelSpeed;
    } else {
        // Right wheel stop
        RIGHT_FORWARD_CV = 0;
        RIGHT_BACKWARD_CV = 0;
    }
}




// Decode function that returns a movementControlMessage
struct movementControlMessage decode_motor_control(uint8_t data) {
    struct movementControlMessage controlMessage;

    // Upper 4 bits (forward/backward)
    int forwardBackward = (int)(data >> 4);
    // Lower 4 bits (left/right)
    int leftRight = (int)(data & 0x0F);

    // Forward/backward
    if (forwardBackward <= 7) {
        // 0000�0111 => backward
        controlMessage.forwardLevel  = 0;
        controlMessage.backwardLevel = 7 - forwardBackward;
    } else if (forwardBackward >= 8 && forwardBackward <= 14) {
        // 1000�1110 => forward
        controlMessage.forwardLevel  = forwardBackward - 8;
        controlMessage.backwardLevel = 0;
    } else {
        // 1111 => unused
        controlMessage.forwardLevel  = 0;
        controlMessage.backwardLevel = 0;
    }

    // Left/right
    if (leftRight <= 7) {
        // 0000�0111 => left
        controlMessage.leftLevel  = 7 - leftRight;
        controlMessage.rightLevel = 0;
    } else if (leftRight >= 8 && leftRight <= 14) {
        // 1000�1110 => right
        controlMessage.rightLevel = leftRight - 8;
        controlMessage.leftLevel  = 0;
    } else {
        // 1111 => unused
        controlMessage.leftLevel  = 0;
        controlMessage.rightLevel = 0;
    }
		
		if (forwardBackward == 15) {
			controlMessage.finish = true;
		}
		else {
			controlMessage.finish = false;
		}

    return controlMessage;
}
