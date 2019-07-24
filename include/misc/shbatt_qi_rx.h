/*
 * Copyright (C) 2018 SHARP CORPORATION All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef SHBATT_QIRX_H
#define SHBATT_QIRX_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

struct wake_lock {
    struct wakeup_source ws;
};

enum {
    WAKE_LOCK_SUSPEND, /* Prevent suspend */
    WAKE_LOCK_TYPE_COUNT
};

enum qirx_state{
    QIRX_IDLE = 0,
    QIRX_INHIBIT,
    QIRX_SUSPEND,
    QIRX_GUIDING,
    QIRX_CHARGING,
    QIRX_FULL,
    QIRX_ERROR,
};

enum print_level{
	QIRX_INFO = BIT(0),
	QIRX_DEBUG = BIT(1),
};

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/
static inline void wake_lock_init(struct wake_lock *lock, int type,
                  const char *name)
{
    wakeup_source_init(&lock->ws, name);
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
    wakeup_source_trash(&lock->ws);
}

static inline void wake_lock(struct wake_lock *lock)
{
    __pm_stay_awake(&lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
    __pm_wakeup_event(&lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
    __pm_relax(&lock->ws);
}

static inline int wake_lock_active(struct wake_lock *lock)
{
    return lock->ws.active;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PRIVATE FUNCTION PROTO TYPE DECLARE :                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_QIRX_H */
