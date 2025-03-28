#include "MKL25Z4.h"
#include "motor.h"

#define LEFT_FORWARD 1
#define LEFT_BACKWARD 0
#define RIGHT_FORWARD 3
#define RIGHT_BACKWARD 2

#define LEFT_FORWARD_CV   TPM0_C0V
#define LEFT_BACKWARD_CV  TPM0_C1V
#define RIGHT_FORWARD_CV  TPM0_C2V
#define RIGHT_BACKWARD_CV TPM0_C3V

void initMotorPWM(void) {
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD->PCR[LEFT_BACKWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[LEFT_BACKWARD] |= PORT_PCR_MUX(4);
    PORTD->PCR[LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[LEFT_FORWARD] |= PORT_PCR_MUX(4);

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

    TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    LEFT_FORWARD_CV = 0;
    LEFT_BACKWARD_CV = 0;
    RIGHT_FORWARD_CV = 0;
    RIGHT_BACKWARD_CV = 0;
}

void movement_master_control(struct movementControlMessage msg) {
    float linearSpeed = (msg.forwardLevel - msg.backwardLevel) / 7.0;
    float angularSpeed = (msg.leftLevel - msg.rightLevel) / 7.0;

    float leftWheelSpeedNormalized = linearSpeed - angularSpeed;
    float rightWheelSpeedNormalized = linearSpeed + angularSpeed;

    int leftWheelSpeed = (int)(7500 * fabs(leftWheelSpeedNormalized));
    int rightWheelSpeed = (int)(7500 * fabs(rightWheelSpeedNormalized));

    if (leftWheelSpeedNormalized > 0) {
        LEFT_FORWARD_CV = leftWheelSpeed;
        LEFT_BACKWARD_CV = 0;
    } else if (leftWheelSpeedNormalized < 0) {
        LEFT_FORWARD_CV = 0;
        LEFT_BACKWARD_CV = leftWheelSpeed;
    } else {
        LEFT_FORWARD_CV = 0;
        LEFT_BACKWARD_CV = 0;
    }

    if (rightWheelSpeedNormalized > 0) {
        RIGHT_FORWARD_CV = rightWheelSpeed;
        RIGHT_BACKWARD_CV = 0;
    } else if (rightWheelSpeedNormalized < 0) {
        RIGHT_FORWARD_CV = 0;
        RIGHT_BACKWARD_CV = rightWheelSpeed;
    } else {
        RIGHT_FORWARD_CV = 0;
        RIGHT_BACKWARD_CV = 0;
    }
}