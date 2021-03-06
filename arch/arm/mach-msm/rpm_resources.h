/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_RPM_RESOURCES_H
#define __ARCH_ARM_MACH_MSM_RPM_RESOURCES_H

#include <mach/rpm.h>
#include "pm.h"


struct msm_rpmrs_limits {
	uint32_t pxo;
	uint32_t l2_cache;
	uint32_t vdd_mem_upper_bound;
	uint32_t vdd_mem;
	uint32_t vdd_dig_upper_bound;
	uint32_t vdd_dig;

	uint32_t latency_us[NR_CPUS];
	uint32_t power[NR_CPUS];
};

enum {
	MSM_RPMRS_PXO_OFF = 0,
	MSM_RPMRS_PXO_ON = 1,
};

enum {
	MSM_RPMRS_L2_CACHE_HSFS_OPEN = 0,
	MSM_RPMRS_L2_CACHE_GDHS = 1,
	MSM_RPMRS_L2_CACHE_RETENTION = 2,
	MSM_RPMRS_L2_CACHE_ACTIVE = 3,
};

enum {
	MSM_RPMRS_VDD_MEM_RET_LOW = 500,
	MSM_RPMRS_VDD_MEM_RET_HIGH = 750,
	MSM_RPMRS_VDD_MEM_ACTIVE = 1000,
	MSM_RPMRS_VDD_MEM_MAX = 1250,
};

enum {
	MSM_RPMRS_VDD_DIG_RET_LOW = 500,
	MSM_RPMRS_VDD_DIG_RET_HIGH = 750,
	MSM_RPMRS_VDD_DIG_ACTIVE = 1000,
	MSM_RPMRS_VDD_DIG_MAX = 1250,
};

#define MSM_RPMRS_LIMITS(_pxo, _l2, _vdd_upper_b, _vdd) { \
	MSM_RPMRS_PXO_##_pxo, \
	MSM_RPMRS_L2_CACHE_##_l2, \
	MSM_RPMRS_VDD_MEM_##_vdd_upper_b, \
	MSM_RPMRS_VDD_MEM_##_vdd, \
	MSM_RPMRS_VDD_DIG_##_vdd_upper_b, \
	MSM_RPMRS_VDD_DIG_##_vdd, \
	{0}, {0}, \
}

struct msm_rpmrs_level {
	enum msm_pm_sleep_mode sleep_mode;
	struct msm_rpmrs_limits rs_limits;
	bool available;
	uint32_t latency_us;
	uint32_t steady_state_power;
	uint32_t energy_overhead;
	uint32_t time_overhead_us;
};

int msm_rpmrs_set(int ctx, struct msm_rpm_iv_pair *req, int count);
int msm_rpmrs_set_noirq(int ctx, struct msm_rpm_iv_pair *req, int count);

static inline int msm_rpmrs_set_nosleep(
	int ctx, struct msm_rpm_iv_pair *req, int count)
{
	unsigned long flags;
	int rc;

	local_irq_save(flags);
	rc = msm_rpmrs_set_noirq(ctx, req, count);
	local_irq_restore(flags);

	return rc;
}

int msm_rpmrs_clear(int ctx, struct msm_rpm_iv_pair *req, int count);
int msm_rpmrs_clear_noirq(int ctx, struct msm_rpm_iv_pair *req, int count);

static inline int msm_rpmrs_clear_nosleep(
	int ctx, struct msm_rpm_iv_pair *req, int count)
{
	unsigned long flags;
	int rc;

	local_irq_save(flags);
	rc = msm_rpmrs_clear_noirq(ctx, req, count);
	local_irq_restore(flags);

	return rc;
}

void msm_rpmrs_show_resources(void);

struct msm_rpmrs_limits *msm_rpmrs_lowest_limits(
	bool from_idle, enum msm_pm_sleep_mode sleep_mode, uint32_t latency_us,
	uint32_t sleep_us);

int msm_rpmrs_enter_sleep(uint32_t sclk_count, struct msm_rpmrs_limits *limits,
		bool from_idle, bool notify_rpm);
void msm_rpmrs_exit_sleep(struct msm_rpmrs_limits *limits,
		bool from_idle, bool notify_rpm);

int msm_rpmrs_levels_init(struct msm_rpmrs_level *levels, int size);

#endif /* __ARCH_ARM_MACH_MSM_RPM_RESOURCES_H */
