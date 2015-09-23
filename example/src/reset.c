#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>

#include <gpio.h>

#include "thread.h"
#include "utils.h"
#include "reset.h"

#define RESET_TICK				(100 * TIME_MILLISECOND)
#define RESET_READY_INTERVAL	(10 * 10)

#define RESET_PIN				GPIO_P8_GPIO_26
#define RESET_PORT				GPIO_PORT_P8

typedef enum {
    RESET_STATE_IDLE = 0x00,
    RESET_STATE_START,
    RESET_STATE_READY,
} reset_state_t;

typedef struct {
    reset_state_t value;
    int counter;
} reset_t;

reset_t reset_state;
thread_config_t reset_thread;

static void * on_thread(void *arg)
{
    reset_t * state = (reset_t *)arg;
    int rc;

    while(1) {
        struct timespec t = {
            .tv_sec = 0x00,
            .tv_nsec = RESET_TICK,
        };
        // 10ms tick
        nanosleep(&t, NULL);
        if ((state->value == RESET_STATE_IDLE) ||
            (state->value == RESET_STATE_START)) {

            // read gpio
            rc = gpio_read(RESET_PORT, RESET_PIN);
            if (rc == 0x00) {
                if (state->value == RESET_STATE_IDLE) {
                    state->value = RESET_STATE_START;
                    state->counter = 0;
                    fprintf(stderr, "reset start !\n");
                } else {
                    state->counter++;
                    if (state->counter >= RESET_READY_INTERVAL) {
                        state->value = RESET_STATE_READY;
                        state->counter = 0x00;
                        fprintf(stderr, "reset ready !\n");
                    }
                }
            } else if (state->value != RESET_STATE_IDLE) {
                state->counter = 0;
                state->value = RESET_STATE_IDLE;
            }
        }
    }
    return NULL;
}

int reset_init(void)
{
    int err;
    err = gpio_init();
    if (err != 0) {
        fprintf(stderr,"error: could not init gpio\n");
        return -1;
    }
    // set led direction (output)
    err = gpio_setdir(RESET_PORT, RESET_PIN, GPIO_DIR_INPUT);
    if (err != 0) {
        fprintf(stderr,"error: could not set reset direction\n");
        return -1;
    }
    reset_state.value = RESET_STATE_IDLE;
    reset_state.counter = 0;

    // init led thread
    err = thread_start(&reset_thread,
                       THREAD_DETACHED,
                       on_thread, &reset_state);
    if (err < 0) {
        fprintf(stderr,"error: could not init reset thread\n");
        return -1;
    }
    return 0;
}

void reset_close(void)
{
    thread_stop(&reset_thread);
    gpio_deinit();
}

int reset_activated(void)
{
    return ((reset_state.value == RESET_STATE_READY) ? 1 : 0);
}

void reset_clear(void)
{
    reset_state.value = RESET_STATE_IDLE;
    reset_state.counter = 0;
}
