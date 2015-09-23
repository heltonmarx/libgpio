#ifndef RESET_H_INCLUDED
#define RESET_H_INCLUDED

/**
 *	Init reset process (detached thread).
 *
 *	@return		Success, zero is returned.
 *	@return		On error, -1 is returned.
 */
int reset_init(void);

/**
 *	Stop reset process (detached thread).
 */
void reset_close(void);

/**
 *	Check if reset is activated.
 *
 * 	@return		Activated, 1 is returned.
 *	@return		Idle, 0 is returned.
 */
int reset_activated(void);

/**
 *	Called after check if reset is activated
 */
void reset_clear(void);

#endif	/* RESET_H_INCLUDED */
