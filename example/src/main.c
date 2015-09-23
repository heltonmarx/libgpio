#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "reset.h"
#include "leds.h"

/**
 *	Read Reset (input): GPIO 26		Port P8
 *	Led #1 (output):	GPIO 44 	Port P8
 *	Led #2 (output):	GPIO 45 	Port P8
 *	Led #3 (output):	GPIO 49		Port P9
 */

int main(void)
{
    int ret;
    int counter = 0;

    ret = reset_init();
    if (ret != 0) {
        return -1;
    }

    ret = led_init();
    if (ret != 0) {
        return -1;
    }

    while(1) {
        struct timespec t = {
            .tv_sec = 1, .tv_nsec = 0,
        };
        // 10ms tick
        nanosleep(&t, NULL);

        if (reset_activated() == 1) {
            reset_clear();
            fprintf(stderr,"reset activated !\n");
        }
        led_set(counter, 125);
        counter++;
        if (counter >= LED_MAX) {
            counter = 0;
        }

    }
    return 0;
}
