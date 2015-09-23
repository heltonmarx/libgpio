#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED

enum {
    GPIO_DIR_INPUT = 0x00,
    GPIO_DIR_OUTPUT,
};

enum {
    GPIO_PORT_P8 = 0x08,
    GPIO_PORT_P9 = 0x09,
};

enum {
    GPIO_OFF = 0,
    GPIO_ON,
};

/**
 *  GPIO P8 Cape Expansion Headers
 */
enum {
    GPIO_P8_DGND        = 0x00, /* unused */
    GPIO_P8_GPIO_38     = 0x02,	/* 03 */
    GPIO_P8_GPIO_39,		/* 04 */
    GPIO_P8_GPIO_34,		/* 05 */
    GPIO_P8_GPIO_35,		/* 06 */
    GPIO_P8_GPIO_66,		/* 07 */
    GPIO_P8_GPIO_67,		/* 08 */
    GPIO_P8_GPIO_68,		/* 09 */
    GPIO_P8_GPIO_69,		/* 10 */
    GPIO_P8_GPIO_45,		/* 11 */
    GPIO_P8_GPIO_44,		/* 12 */
    GPIO_P8_GPIO_23,		/* 13 */
    GPIO_P8_GPIO_26,		/* 14 */
    GPIO_P8_GPIO_47,		/* 15 */
    GPIO_P8_GPIO_46,		/* 16 */
    GPIO_P8_GPIO_27,		/* 17 */
    GPIO_P8_GPIO_65,		/* 18 */
    GPIO_P8_GPIO_22,		/* 19 */
    GPIO_P8_GPIO_63,		/* 20 */
    GPIO_P8_GPIO_62,		/* 21 */
    GPIO_P8_GPIO_37,		/* 22 */
    GPIO_P8_GPIO_36,		/* 23 */
    GPIO_P8_GPIO_33,		/* 24 */
    GPIO_P8_GPIO_32,		/* 25 */
    GPIO_P8_GPIO_61,		/* 26 */
    GPIO_P8_GPIO_86,		/* 27 */
    GPIO_P8_GPIO_88,		/* 28 */
    GPIO_P8_GPIO_87,		/* 29 */
    GPIO_P8_GPIO_89,		/* 30 */
    GPIO_P8_GPIO_10,		/* 31 */
    GPIO_P8_GPIO_11,		/* 32 */
    GPIO_P8_GPIO_09,		/* 33 */
    GPIO_P8_GPIO_81,		/* 34 */
    GPIO_P8_GPIO_08,		/* 35 */
    GPIO_P8_GPIO_80,		/* 36 */
    GPIO_P8_GPIO_78,		/* 37 */
    GPIO_P8_GPIO_79,		/* 38 */
    GPIO_P8_GPIO_76,		/* 39 */
    GPIO_P8_GPIO_77,		/* 40 */
    GPIO_P8_GPIO_74,		/* 41 */
    GPIO_P8_GPIO_75,		/* 42 */
    GPIO_P8_GPIO_72,		/* 43 */
    GPIO_P8_GPIO_73,		/* 44 */
    GPIO_P8_GPIO_70,		/* 45 */
    GPIO_P8_GPIO_71,		/* 46 */
    GPIO_P8_MAX = GPIO_P8_GPIO_71,	/* 46 */
};

/**
 *  GPIO P9 Cape Expansion Headers
 */
enum {
    GPIO_P9_DGND        = 0x00, /* unused */
    GPIO_P9_3V3         = 0x02, /* unused */
    GPIO_P9_5V          = 0x04, /* unused */
    GPIO_P9_SYS_5V      = 0x06, /* unused */
    GPIO_P9_PWR_BUT     = 0x08,
    GPIO_P9_SYS_RESET,  /* [10] */
    GPIO_P9_GPIO_30,    /* [11] */
    GPIO_P9_GPIO_60,    /* [12] */
    GPIO_P9_GPIO_31,    /* [13] */
    GPIO_P9_GPIO_50,    /* [14] */
    GPIO_P9_GPIO_48,    /* [15] */
    GPIO_P9_GPIO_51,    /* [16] */
    GPIO_P9_GPIO_05,    /* [17] */
    GPIO_P9_GPIO_04,    /* [18] */
    GPIO_P9_I2C2_SCL,   /* [19] */
    GPIO_P9_I2C2_SDA,   /* [20] */
    GPIO_P9_GPIO_3,     /* [21] */
    GPIO_P9_GPIO_2,     /* [22] */
    GPIO_P9_GPIO_49,    /* [23] */
    GPIO_P9_GPIO_15,    /* unused */
    GPIO_P9_GPIO_117,   /* [25] */
    GPIO_P9_GPIO_14,    /* [26] */
    GPIO_P9_GPIO_115,   /* [27] */
    GPIO_P9_GPIO_123,   /* [28] */
    GPIO_P9_GPIO_121,   /* [29] */
    GPIO_P9_GPIO_122,   /* [30] */
    GPIO_P9_GPIO_120,   /* [31] */
    GPIO_P9_VCC_ADC,    /* unused */
    GPIO_P9_AIN4,       /* [33] */
    GPIO_P9_GNDA_ADC,   /* unused */
    GPIO_P9_AIN6,       /* [35] */
    GPIO_P9_AIN5,       /* [36] */
    GPIO_P9_AIN2,       /* [37] */
    GPIO_P9_AIN3,       /* [38] */
    GPIO_P9_AIN0,       /* [39] */
    GPIO_P9_AIN1,       /* [40] */
    GPIO_P9_GPIO_20,    /* [41] */
    GPIO_P9_GPIO_07,    /* [42] */
    GPIO_P9_MAX = (GPIO_P9_GPIO_07 + 4),
};

/**
 *	Init and Open BeagleBone GPIO access
 *
 *	@return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int gpio_init(void);

/**
 *	Close BeagleBone GPIO access.
 *
 *	@return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int gpio_deinit(void);

/**
 *	Set port direction (GPIO_DIR_INPUT, GPIO_DIR_OUTPUT), where port
 *	is 8 or 9 and pin is 1 up to 46.
 *
 *	@param [in]	port		GPIO_PORT_P8 or GPIO_PORT_P9.
 *	@param [in] pin			0 up to 45.
 *	@param [in]	direction	GPIO_DIR_INPUT or GPIO_DIR_OUTPUT.
 *
 *	@return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int gpio_setdir(int port, int pin, int direction);

/**
 *	Read the actual value of input gpio.
 *
 *	@param [in] port	GPIO_PORT_P8 or GPIO_PORT_P9.
 *	@param [in] pin		0 up to 45.
 *
 * 	@return		Success, value of gpio(1 or 0) is returned.
 * 	@return		On error, -1 is returned.
 */
int gpio_read(int port, int pin);

/**
 *	Set status of gpio output (ON or OFF).
 *
 *	@param [in] port	GPIO_PORT_P8 or GPIO_PORT_P9.
 *	@param [in] pin		0 up to 45.
 *	@param [in] status	GPIO_OFF or GPIO_ON
 *
 *	@return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int gpio_write(int port, int pin, int status);

#endif	/* GPIO_H_INCLUDED */
