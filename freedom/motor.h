#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MIN_SPEED 700
#define MAX_SPEED 20000

struct movementControlMessage {
    int forwardLevel;   // Level of forward movement (0-7)
    int backwardLevel;  // Level of backward movement (0-7)
    int leftLevel;      // Level of left movement (0-7)
    int rightLevel;     // Level of right movement (0-7)
};

void initMotorPWM(void);
void movement_master_control(struct movementControlMessage msg);

// Function that returns the decoded movementControlMessage
struct movementControlMessage decode_motor_control(uint8_t data);

#endif // MOTOR_H
