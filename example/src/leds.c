#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <gpio.h>

#include "thread.h"
#include "utils.h"
#include "leds.h"

#define LED_ON			GPIO_OFF
#define LED_OFF			GPIO_ON

typedef struct {
    int pin;
    int port;
    int status;
    int interval;
} led_t;

static led_t led[] = {
    {.pin = GPIO_P8_GPIO_44, .port = GPIO_PORT_P8, .status = LED_OFF, .interval = 0},
    {.pin = GPIO_P8_GPIO_45, .port = GPIO_PORT_P8, .status = LED_OFF, .interval = 0},
    {.pin = GPIO_P9_GPIO_49, .port = GPIO_PORT_P9, .status = LED_OFF, .interval = 0},
};

#define LED_TICK				(10 * TIME_MILLISECOND)

thread_config_t leds_thread;

void led_on(int index)
{
    if (index < LED_MAX) {
        if (led[index].status != LED_ON) {
            gpio_write(led[index].port, led[index].pin, LED_ON);
            led[index].status = LED_ON;
        }
    }
}

void led_off(int index)
{
    if (index < LED_MAX) {
        if (led[index].status != LED_OFF) {
            gpio_write(led[index].port, led[index].pin, LED_OFF);
            led[index].status = LED_OFF;
        }
    }
}

void *process(void *arg)
{
    int i;
    UNUSED_ARG(arg);

    while(1) {
        struct timespec t = {
            .tv_sec = 0x00,
            .tv_nsec = LED_TICK,
        };
        nanosleep(&t, NULL);

        // execute at time interval
        for (i = 0; i < LED_MAX; i++) {
            if (led[i].interval > 0) {
                led[i].interval--;
            } else if (led[i].status != LED_ON) {
                gpio_write(led[i].port, led[i].pin, LED_ON);
                led[i].status = LED_ON;
            }
        }
    }
    return NULL;
}

int led_init(void)
{
    int err;
    int i;

    err = gpio_init();
    if (err != 0) {
        fprintf(stderr,"error: could not init gpio\n");
        return -1;
    }
    // set led direction (output)
    for (i = 0; i < LED_MAX; i++) {
        err = gpio_setdir(led[i].port, led[i].pin, GPIO_DIR_OUTPUT);
        if (err != 0) {
            fprintf(stderr,"error: could not set led direction\n");
            return -1;
        }
        gpio_write(led[i].port, led[i].pin, LED_ON);
        led[i].status = LED_ON;
    }
    // init led thread
    err = thread_start(&leds_thread,
                       THREAD_DETACHED,
                       process, NULL);
    if (err < 0) {
        fprintf(stderr,"error: could not init led thread\n");
        return -1;
    }
    return 0;
}

void led_deinit(void)
{
    // stop led thread
    thread_stop(&leds_thread);

    // deinit gpio
    gpio_deinit();
}

void led_set(int index, int value)
{
    if ((index < LED_MAX) && (value > 0)) {
        if (led[index].status != LED_OFF) {
            gpio_write(led[index].port, led[index].pin, LED_OFF);
            led[index].status = LED_OFF;
            led[index].interval = value;
        }
    }
}
