#ifndef LED_H
#define LED_H

void initLED(void);
void red_led_thread(void *argument);
void green_led_thread(void *argument);

#endif // LED_H