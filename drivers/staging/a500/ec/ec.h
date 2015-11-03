/*
 * MFD driver for Acer A50x embedded controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_A500_EC_H
#define __MFD_A500_EC_H

struct ec_reg_data {
	u8  addr;
	u16 timeout;
};

#define EC_REG_DATA(_name, _addr, _timeout)		\
static struct ec_reg_data ec_##_name##_ = {		\
	.addr = _addr,					\
	.timeout = _timeout,				\
};							\
static struct ec_reg_data *_name = &ec_##_name##_;

extern int ec_read_word_data_locked(struct ec_reg_data *reg_data);
extern int ec_read_word_data(struct ec_reg_data *reg_data);
extern int ec_write_word_data_locked(struct ec_reg_data *reg_data, u16 value);
extern int ec_write_word_data(struct ec_reg_data *reg_data, u16 value);
extern void ec_lock(void);
extern void ec_unlock(void);

#endif	/* __MFD_A500_EC_H */
