/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 *  Copyright (C) 2021-2025 Masahito Suzuki <firelzrd@gmail.com>
 */
#ifndef _KERNEL_SCHED_BORE_H
#define _KERNEL_SCHED_BORE_H

#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/jump_label.h>
#include <linux/sysctl.h>

#define SCHED_BORE_AUTHOR   "Masahito Suzuki"
#define SCHED_BORE_PROGNAME "BORE CPU Scheduler modification"

#define SCHED_BORE_VERSION  "7.0.1r2"

#ifdef CONFIG_SCHED_BORE
#define BORE_BC_TIMESTAMP_SHIFT 16

struct bore_bc {
	union {
		struct {
			u64 timestamp:48;
			u64 penalty:16;
		};
		u64 value;
	};
};

struct bore_ctx {
	u64 burst_time;
	u64 credit_sleep;
	u16 prev_penalty;
	u16 curr_penalty;
	u16 penalty;
	bool stop_update;
	bool futex_waiting;
	struct bore_bc subtree;
	struct bore_bc group;
};

extern unsigned int __read_mostly sched_bore;
DECLARE_STATIC_KEY_TRUE(sched_bore_key);
extern unsigned int __read_mostly sched_burst_inherit_type;
extern unsigned int __read_mostly sched_burst_smoothness;
extern unsigned int __read_mostly sched_burst_penalty_offset;
extern unsigned int __read_mostly sched_burst_penalty_scale;
extern unsigned int __read_mostly sched_burst_cache_lifetime;

extern unsigned int __read_mostly sched_credit_cap_us;
DECLARE_STATIC_KEY_FALSE(sched_credit_key);
DECLARE_STATIC_KEY_TRUE(sched_burst_protect_slice_cond_key);
DECLARE_STATIC_KEY_FALSE(sched_burst_protect_slice_prefer_key);
extern unsigned int sched_burst_protect_slice_lv;
extern unsigned int sysctl_sched_min_base_slice;


extern void bore_note_sleep(struct task_struct *p, u64 now);
extern u64  bore_credit_ns(struct task_struct *p);

static inline u8 bore_score(struct task_struct *p)
{ return READ_ONCE(p->bore->penalty) >> 8; }

extern u8   effective_prio_bore(struct task_struct *p);
extern void update_curr_bore(struct task_struct *p, u64 delta_exec);
extern void restart_burst_bore(struct task_struct *p);
extern void restart_burst_rescale_deadline_bore(struct task_struct *p);
extern void task_fork_bore(struct task_struct *p, struct task_struct *parent,
			   u64 clone_flags, u64 now);
extern void sched_init_bore(void);
extern void reset_task_bore(struct task_struct *p);

extern int  sched_bore_update_handler(struct ctl_table *table,
	int write, void *buffer, size_t *lenp, loff_t *ppos);
extern int  sched_burst_inherit_type_update_handler(struct ctl_table *table,
	int write, void *buffer, size_t *lenp, loff_t *ppos);
extern int  sched_credit_cap_us_update_handler(struct ctl_table *table,
	int write, void *buffer, size_t *lenp, loff_t *ppos);

extern int sched_burst_protect_slice_update_handler(struct ctl_table *table,
	int write, void *buffer, size_t *lenp, loff_t *ppos);
extern int sched_min_base_slice_update_handler(struct ctl_table *table,
	int write, void *buffer, size_t *lenp, loff_t *ppos);

int bore_alloc_task(struct task_struct *p, int node);
void bore_free_task(struct task_struct *p);
#else
static inline int bore_alloc_task(struct task_struct *p, int node) { return 0; }
static inline void bore_free_task(struct task_struct *p) { }
#endif /* CONFIG_SCHED_BORE */

#endif /* _KERNEL_SCHED_BORE_H */
