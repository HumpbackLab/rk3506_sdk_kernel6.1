/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_PWM_WAVE_H
#define _UAPI_LINUX_PWM_WAVE_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define PWM_WAVE_ABI_VERSION	1

#define PWM_WAVE_FLAG_16BIT	0x00000001U

/**
 * struct pwm_wave_caps - immutable wave generator capabilities
 * @abi_version: PWM wave userspace ABI version
 * @max_entries_8bit: maximum number of 8-bit lookup-table entries
 * @max_entries_16bit: maximum number of 16-bit lookup-table entries
 * @supported_flags: supported PWM_WAVE_FLAG_* values
 * @reserved: reserved for future use, must be zero
 */
struct pwm_wave_caps {
	__u32 abi_version;
	__u32 max_entries_8bit;
	__u32 max_entries_16bit;
	__u32 supported_flags;
	__u32 reserved[4];
};

/**
 * struct pwm_wave_config - runtime waveform configuration
 * @period_ns: fixed output period in nanoseconds
 * @clock_rate_hz: requested wave generator clock rate
 * @repeat: repeat each lookup-table entry N + 1 periods
 * @flags: PWM_WAVE_FLAG_* values
 * @reserved: reserved for future use, must be zero
 */
struct pwm_wave_config {
	__aligned_u64 period_ns;
	__aligned_u64 clock_rate_hz;
	__u32 repeat;
	__u32 flags;
	__u32 reserved[4];
};

#define PWM_WAVE_IOC_MAGIC	'P'
#define PWM_WAVE_IOC_GET_CAPS	_IOR(PWM_WAVE_IOC_MAGIC, 0x00, \
				     struct pwm_wave_caps)
#define PWM_WAVE_IOC_SET_CONFIG	_IOW(PWM_WAVE_IOC_MAGIC, 0x01, \
				     struct pwm_wave_config)
#define PWM_WAVE_IOC_GET_CONFIG	_IOR(PWM_WAVE_IOC_MAGIC, 0x02, \
				     struct pwm_wave_config)
#define PWM_WAVE_IOC_STOP	_IO(PWM_WAVE_IOC_MAGIC, 0x03)

#endif /* _UAPI_LINUX_PWM_WAVE_H */
