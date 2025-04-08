#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define MIN_SPEED 1500
#define MAX_SPEED 5800

#define LEFT_BIAS		20
#define RIGHT_BIAS 	0

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

struct movementControlMessage {
    int forwardLevel;   // Level of forward movement (0-7)
    int backwardLevel;  // Level of backward movement (0-7)
    int leftLevel;      // Level of left movement (0-7)
    int rightLevel;     // Level of right movement (0-7)
		bool finish;
};

extern struct movementControlMessage idle;
extern  struct movementControlMessage forward;
extern  struct movementControlMessage backward; // = {0, 7, 0, 0};
extern  struct movementControlMessage left; // = {0, 0, 7, 0};
 extern struct movementControlMessage right; // = {0, 0, 0, 7};


void initMotorPWM(void);
void movement_master_control(struct movementControlMessage msg);

// Function that returns the decoded movementControlMessage
struct movementControlMessage decode_motor_control(uint8_t data);

#endif // MOTOR_H
