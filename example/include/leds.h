#ifndef LEDS_H_INCLUDED
#define LEDS_H_INCLUDED

enum {
    LED_RF  = 0x00,	// ZigBee
    LED_IR,	// infrared
    LED_HB,	// heartbeat
    LED_MAX
};
/**
 *	Init led process (thread)
 *
 *  @return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int led_init(void);

/**
 * 	Stop led process (thread).
 */
void led_deinit();

/**
 *	Set Led on by an interval (*10 ms).
 *
 * 	@param index	led index (LED_RF, LED_IR, LED_HB)
 *	@param value	time interval (*10 ms)
 */
void led_set(int index, int value);

/**
 *	Set Led on.
 *
 *	@param index	led index (LED_RF, LED_IR, LED_HB)
 */
void led_on(int index);

/**
 *	Set Led off.
 *
 *	@param index	led index (LED_RF, LED_IR, LED_HB)
 */
void led_off(int index);

#endif	/* LEDS_H_INCLUDED */
