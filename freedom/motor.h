#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MIN_SPEED 700
#define MAX_SPEED 20000

struct movementControlMessage {
    uint8_t forwardLevel;   // Level of forward movement (0-7)
    uint8_t backwardLevel;  // Level of backward movement (0-7)
    uint8_t leftLevel;      // Level of left movement (0-7)
    uint8_t rightLevel;     // Level of right movement (0-7)
};

void initMotorPWM(void);
void movement_master_control(struct movementControlMessage msg);

#endif // MOTOR_H