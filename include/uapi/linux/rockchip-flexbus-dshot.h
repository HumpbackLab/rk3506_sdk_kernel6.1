/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ROCKCHIP_FLEXBUS_DSHOT_H
#define _UAPI_LINUX_ROCKCHIP_FLEXBUS_DSHOT_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define RK_DSHOT_CHANNELS		4

struct rk_dshot_frame {
	__u16 value[RK_DSHOT_CHANNELS];
};

struct rk_dshot_passthrough_xfer {
	__u64 tx_buf;
	__u64 rx_buf;
	__u32 tx_samples;
	__u32 rx_samples;
	__u32 sample_rate;
	__u32 channel;
	__u32 timeout_ms;
	__u32 flags;
};

/*
 * erpm[] is electrical RPM.  Mechanical RPM depends on the motor pole count:
 * mechanical_rpm = erpm * 2 / pole_count.
 */
struct rk_dshot_telemetry {
	__u32 erpm[RK_DSHOT_CHANNELS];
	__u16 raw[RK_DSHOT_CHANNELS];
	__u8 valid_mask;
	__u8 no_response_mask;
	__u16 reserved;
	__u32 packet_count[RK_DSHOT_CHANNELS];
	__u32 error_count[RK_DSHOT_CHANNELS];
};

struct rk_dshot_telemetry_xfer {
	struct rk_dshot_frame frame;
	struct rk_dshot_telemetry telemetry;
};

#define RK_DSHOT_IOCTL_BASE		'D'
#define RK_DSHOT_IOC_SET_RATE		_IOW(RK_DSHOT_IOCTL_BASE, 0x00, __u32)
#define RK_DSHOT_IOC_SET_TELEMETRY	_IOW(RK_DSHOT_IOCTL_BASE, 0x01, __u32)
#define RK_DSHOT_IOC_SEND_CMD		_IOW(RK_DSHOT_IOCTL_BASE, 0x02, __u32)
#define RK_DSHOT_IOC_SEND_FRAME		_IOW(RK_DSHOT_IOCTL_BASE, 0x03, \
					     struct rk_dshot_frame)
#define RK_DSHOT_IOC_PASSTHROUGH_XFER	_IOWR(RK_DSHOT_IOCTL_BASE, 0x04, \
					      struct rk_dshot_passthrough_xfer)
#define RK_DSHOT_IOC_GET_TELEMETRY	_IOR(RK_DSHOT_IOCTL_BASE, 0x05, \
					     struct rk_dshot_telemetry)
#define RK_DSHOT_IOC_TELEMETRY_XFER	_IOWR(RK_DSHOT_IOCTL_BASE, 0x06, \
					      struct rk_dshot_telemetry_xfer)

#define RK_DSHOT_PASSTHROUGH_F_TX_INVERT	(1U << 0)
#define RK_DSHOT_PASSTHROUGH_F_RX_INVERT	(1U << 1)

#endif /* _UAPI_LINUX_ROCKCHIP_FLEXBUS_DSHOT_H */
