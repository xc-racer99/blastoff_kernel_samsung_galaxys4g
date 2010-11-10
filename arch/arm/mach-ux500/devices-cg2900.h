/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Par-Gunnar Hjalmdahl <par-gunnar.p.hjalmdahl@stericsson.com>
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __DEVICES_CG2900_H
#define __DEVICES_CG2900_H

#include <linux/mfd/cg2900.h>

/**
 * dcg2900_init_platdata() - Initializes platform data with callback functions.
 * @data:	Platform data.
 */
extern void dcg2900_init_platdata(struct cg2900_platform_data *data);

#endif /* __DEVICES_CG2900_H */
