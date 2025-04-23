// motor.c
// Implements PWM setup and motor‑control logic.

#include "MKL25Z4.h"
#include "motor.h"
#include <math.h>

// Predefined command messages
struct movementControlMessage idle     = {0,0,0,0};
struct movementControlMessage forward  = {7,0,0,0};
struct movementControlMessage backward = {0,7,0,0};
struct movementControlMessage left     = {0,0,7,0};
struct movementControlMessage right    = {0,0,0,7};

// Initialize TPM0 channels for four‑way PWM motor outputs
void initMotorPWM(void) {
    // Enable PORTD and TPM0 clocks
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Configure pins for TPM0 CH0–CH3
    PORTD->PCR[LEFT_FORWARD]   = PORT_PCR_MUX(4);
    PORTD->PCR[LEFT_BACKWARD]  = PORT_PCR_MUX(4);
    PORTD->PCR[RIGHT_FORWARD]  = PORT_PCR_MUX(4);
    PORTD->PCR[RIGHT_BACKWARD] = PORT_PCR_MUX(4);

    // Select TPM clock source
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(1);

    // Set PWM frequency (MOD) and start counter
    TPM0->MOD = 6000;  // period
    TPM0->SC  = TPM_SC_CMOD(1) | TPM_SC_PS(7);

    // Configure each channel for edge‑aligned PWM, high‑true pulses
    for (int ch = 0; ch < 4; ch++) {
        TPM0->CONTROLS[ch].CnSC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
        TPM0->CONTROLS[ch].CnV  = 0;  // start at 0% duty
    }
}

// Map movementControlMessage into left/right wheel PWM and direction
void movement_master_control(struct movementControlMessage msg) {
    const float NEUTRAL_THRESHOLD = 0.1f;
    // Use tighter steering sensitivity when not moving forward/back
    float steering_sens = (msg.forwardLevel == 0 && msg.backwardLevel == 0) ? 0.4f : 0.8f;

    // Normalize levels to –1.0 … +1.0
    float lin = (msg.forwardLevel  - msg.backwardLevel) / 7.0f;
    float ang = (msg.leftLevel     - msg.rightLevel)    / 7.0f;

    if (fabsf(lin) < NEUTRAL_THRESHOLD) lin = 0;
    if (fabsf(ang) < NEUTRAL_THRESHOLD) ang = 0;
    ang *= steering_sens;

    float leftN  = lin - ang;
    float rightN = lin + ang;

    // Scale normalized speed into PWM compare value
    int leftV  = (int)(leftN  * MAX_SPEED);
    int rightV = (int)(rightN * MAX_SPEED);

    // Apply to left motor
    if (leftN > 0) {
        LEFT_FORWARD_CV  = leftV + LEFT_BIAS;
        LEFT_BACKWARD_CV = 0;
    } else if (leftN < 0) {
        LEFT_FORWARD_CV  = 0;
        LEFT_BACKWARD_CV = -leftV - LEFT_BIAS;
    } else {
        LEFT_FORWARD_CV = LEFT_BACKWARD_CV = 0;
    }

    // Apply to right motor (same idea)
    if (rightN > 0) {
        RIGHT_FORWARD_CV  = rightV + RIGHT_BIAS;
        RIGHT_BACKWARD_CV = 0;
    } else if (rightN < 0) {
        RIGHT_FORWARD_CV  = 0;
        RIGHT_BACKWARD_CV = -rightV - RIGHT_BIAS;
    } else {
        RIGHT_FORWARD_CV = RIGHT_BACKWARD_CV = 0;
    }
}

// Decode a single-byte command into levels and “finish” flag
struct movementControlMessage decode_motor_control(uint8_t data) {
    struct movementControlMessage m = {0,0,0,0};
    int fb = data >> 4;    // upper nibble = forward/backward
    int lr = data & 0x0F;  // lower nibble = left/right

    if (fb <= 7)       m.backwardLevel = 7 - fb;
    else if (fb <= 14) m.forwardLevel  = fb - 8;

    if (lr <= 7)       m.leftLevel  = 7 - lr;
    else if (lr <= 14) m.rightLevel = lr - 8;

    m.finish = (fb == 15);
    return m;
}
