#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
/** open */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
/** mmap */
#include <sys/mman.h>

#include "gpio.h"

#define CONTROL_MODULE 		    0x44e10000
#define CONTROL_LEN 		    0xA00

#define GPIO_ADDRESS_PORTA	    0x44e07000
#define GPIO_ADDRESS_PORTB	    0x4804c000
#define GPIO_ADDRESS_PORTC	    0x481ac000
#define GPIO_ADDRESS_PORTD	    0x481ae000
#define GPIO_ADDRESS_MAX	    4

#define GPIO_LENGHT			    0x1000
#define GPIO_OE				    0x134
#define GPIO_SETDATAOUT 	    0x194
#define GPIO_CLEARDATAOUT 	    0x190
#define GPIO_DATAIN 		    0x138

typedef struct {
    int fd;
    unsigned int *address[GPIO_ADDRESS_MAX];
} gpio_data_t;

const unsigned int ioregion_base[] = {
    GPIO_ADDRESS_PORTA,
    GPIO_ADDRESS_PORTB,
    GPIO_ADDRESS_PORTC,
    GPIO_ADDRESS_PORTD,
};

/**
 *  P8 bank and bitmask
 */
const int gpio_p8_bank[]= {
    -1,-1,  // DGND,    DGND
    1, 1,   // GPIO_38, GPIO_39
    1, 1,   // GPIO_34, GPIO_35,
    2, 2,   // GPIO_66, GPIO_67
    2, 2,   // GPIO_68, GPIO_69
    1, 1,   // GPIO_45, GPIO_44
    0, 0,   // GPIO_23, GPIO_26
    1, 1,   // GPIO_47, GPIO_46
    0, 2,   // GPIO_27, GPIO_65
    0, 1,   // GPIO_22, GPIO_63
    1, 1,   // GPIO_62, GPIO_37
    1, 1,   // GPIO_36, GPIO_33
    1, 1,   // GPIO_32, GPIO_61
    2, 2,   // GPIO_86, GPIO_88
    2, 2,   // GPIO_87, GPIO_89
    0, 0,   // GPIO_10, GPIO_11
    0, 2,   // GPIO_09, GPIO_81
    0, 2,   // GPIO_08, GPIO_80
    2, 2,   // GPIO_78, GPIO_79
    2,-1,   // GPIO_76, GPIO_77
    2, 2,   // GPIO_74, GPIO_75
    2, 2,   // GPIO_72, GPIO_73
    2, 2    // GPIO_70. GPIO_71
};

const unsigned int gpio_p8_bitmask[]= {
    0,		0,
    1<<6,	1<<7,
    1<<2,	1<<3,
    1<<2,	1<<3,
    1<<5,	1<<4,
    1<<13,	1<<12,
    1<<23,	1<<26,
    1<<15,	1<<14,
    1<<27,	1<<1,
    1<<22,	1<<31,
    1<<30,	1<<5,
    1<<4,	1<<1,
    1<<0,	1<<29,
    1<<22,	1<<24,
    1<<23,	1<<25,
    1<<10,	1<<11,
    1<<9,	1<<17,
    1<<8,	1<<16,
    1<<14,	1<<15,
    1<<12,	0,
    1<<10,	1<<11,
    1<<8,	1<<9,
    1<<6,	1<<7
};

/**
 *  P9 bank and bitmask
 */
const int gpio_p9_bank[]= {
    -1,	-1,	    // DGND         DGND
    -1,	-1,	    // VDD_3V3      VDD_3V3
    -1,	-1,	    // VDD_5V       VDD_5V
    -1,	-1,     // SYS_5V       SYS_5V
    -1,	-1,	    // PWR_BUT,     SYS_RESETn
    0,	 1,	    // GPIO_30,     GPIO_60
    0,	 1,     // GPIO_31,     GPIO_50
    1,	 1,     // GPIO_48,     GPIO_51
    0,	 0,     // GPIO_5,      GPIO_4
    0,	 0,     // I2C2_SCL,    I2C2_SDA
    0,	 0,     // GPIO_3,      GPIO_2
    1,	-1,     // GPIO_49      GPIO_15
    3,	-1,     // GPIO_117     GPIO_14
    3,	 3,     // GPIO_115     GPIO_123
    3,	-1,     // GPIO_121     GPIO_122
    3,	-1,     // GPIO_120     VCC_ADC
    -1,	-1,     // AIN4         GNDA_ADC
    -1,	-1,     // AIN6         AIN5
    -1,	-1,     // AIN2         AIN3
    -1,	-1,     // AIN0         AIN1
    0,	 0,     // GPIO_20      GPIO_7
    -1,	-1,     // DGND,        DGND
    -1,	-1      // DGND,        DGND
};

const unsigned int gpio_p9_bitmask[]= {
    0,		0,		//DGND	DGND
    0,		0,		// VDD_3V3      VDD_3V3
    0,		0,		// VDD_5V       VDD_5V
    0,		0,		// SYS_5V       SYS_5V
    0,		0,		// PWR_BUT,     SYS_RESETn
    1<<30,	1<<28,	// GPIO_30,     GPIO_60
    1<<31,	1<<18,	// GPIO_31,     GPIO_50
    1<<16,	1<<19,	// GPIO_48,     GPIO_51
    1<<5,	1<<4,	// GPIO_5,      GPIO_4
    1<<13,	1<<12,	// I2C2_SCL,    I2C2_SDA
    1<<3,	1<<2,	// GPIO_3,      GPIO_2
    1<<17,	0,		// GPIO_49      GPIO_15
    1<<21,	0,		// GPIO_117     GPIO_14
    1<<19,	1<<17,	// GPIO_115     GPIO_123
    1<<15,	0,		// GPIO_121     GPIO_122
    1<<14,	0,		// GPIO_120     VCC_ADC
    0,     	0,		// AIN4         GNDA_ADC
    0,     	0,		// AIN6         AIN5
    0,     	0,		// AIN2         AIN3
    0,     	0,		// AIN0         AIN1
    1<<20,	1<<7,	// GPIO_20      GPIO_7
    0,     	0,		// DGND,        DGND
    0,     	0		// DGND,        DGND
};

volatile gpio_data_t gpio;

static int check_params(int port, int pin)
{
    if ((port != GPIO_PORT_P8) && (port != GPIO_PORT_P9)) {
        return -1;
    }
    if ((pin > GPIO_P8_MAX) || (pin > GPIO_P9_MAX)) {
        return -1;
    }
    return 0;
}

static int get_index(int port, int pin)
{
    switch(port) {
    case GPIO_PORT_P8:
        return gpio_p8_bank[pin];
    case GPIO_PORT_P9:
        return gpio_p9_bank[pin];
    default:
        return -1;
    }
}

static const unsigned int * get_bitmask(int port)
{
    switch(port) {
    case GPIO_PORT_P8:
        return gpio_p8_bitmask;
    case GPIO_PORT_P9:
        return gpio_p9_bitmask;
    default:
        return NULL;
    }
}

int gpio_init(void)
{
    int i;

    gpio.fd = open("/dev/mem", O_RDWR);
    if (gpio.fd < 0) {
        fprintf(stderr,"open /dev/mem error(%s)\n", strerror(errno));
        return -1;
    }
    /**
     *  gpio map address
     */

    for (i = 0; i < GPIO_ADDRESS_MAX; i++) {
        gpio.address[i] = mmap(0, GPIO_LENGHT,  (PROT_READ | PROT_WRITE),
                               MAP_SHARED, gpio.fd, ioregion_base[i]);
        if (gpio.address[i] == MAP_FAILED) {
            fprintf(stderr,"error: mmap failure (%s)\n", strerror(errno));
            goto on_error;
        }
    }
    return 0;

on_error:
    close(gpio.fd);
    return -1;
}

int gpio_deinit(void)
{
    close(gpio.fd);
    return 0;
}

int gpio_setdir(int port, int pin, int direction)
{
    if (check_params(port, pin)) {
        return -1;
    }
    const unsigned int *bitmask = get_bitmask(port);
    register volatile unsigned int *r;
    volatile int position;

    position = get_index(port, pin);
    if ((position < 0) || (position >= GPIO_ADDRESS_MAX)) {
        fprintf(stderr,"error: invalid position: %d\n", position);
        return -1;
    }
    /** set register address */
    r = (void*)gpio.address[position] + GPIO_OE;
    switch(direction) {
    case GPIO_DIR_OUTPUT:
        *r &= ~(bitmask[pin]);
        break;
    case GPIO_DIR_INPUT:
        *r |= (bitmask[pin]);
        break;
    default:
        fprintf(stderr,"error: invalid direction %d\n", direction);
        return -1;
    }
    return 0;
}

int gpio_read(int port, int pin)
{
    if (check_params(port, pin)) {
        return -1;
    }
    const unsigned int *bitmask = get_bitmask(port);
    volatile int position;
    int ret;

    position = get_index(port, pin);
    if ((position < 0) || (position >= GPIO_ADDRESS_MAX)) {
        fprintf(stderr,"error: invalid position: %d\n", position);
        return -1;
    }
    ret = (*((unsigned int *)((void *)gpio.address[position] + GPIO_DATAIN)) & bitmask[pin]);
    return (ret == 0 ? GPIO_OFF : GPIO_ON);
}

int gpio_write(int port, int pin, int status)
{
    if (check_params(port, pin)) {
        return -1;
    }
    const unsigned int *bitmask = get_bitmask(port);
    volatile int position;

    position = get_index(port, pin);
    if ((position < 0) || (position >= GPIO_ADDRESS_MAX)) {
        fprintf(stderr,"error: invalid position: %d\n", position);
        return -1;
    }
    status = ((status == GPIO_ON) ? GPIO_SETDATAOUT : GPIO_CLEARDATAOUT);
    *((unsigned int *)((void *)gpio.address[position] + status)) =  bitmask[pin];
    return 0;
}

