/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BBQOVN_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BBQOVN_H_

/**
 * @brief Heart Rate Service (BBQOVN)
 * @defgroup bt_bbqovn Heart Rate Service (BBQOVN)
 * @ingroup bluetooth
 * @{
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//	8800c4fb-a175-b666-2ebb-1e97e6fa47b3
#define BBQOVN_UUID_BASE   			0xb3,0x47,0xfa,0xe6,0x97,0x1e,0xbb,0x2e,0x66,0xb6,0x75,0xa1,0xfb,0xc4,0x00,0x88
//	0000ffe1-0000-1000-8000-00805f9b34fb
#define BBQOVN_CHAR_UUID_SENSOR   	0xffe1
//	4c0a0ff9-1c6d-1c49-e07f-10f1c55905ca
#define BBQOVN_NOTIFY_UUID_BASE   	0xca,0x05,0x59,0xc5,0xf1,0x10,0x7f,0xe0,0x49,0x1c,0x6d,0x1c,0xf9,0x0f,0x0a,0x4c,

//	0000ffe0-0000-1000-8000-00805f9b34fb
#define DX2003_UUID_BASE   			0xffe0
//	0000ffe0-0000-1000-8000-00805f9b34fb
#define DX2003_NOTIFY_UUID_BASE   	0xffe2

struct bbqovn_test_recv_t {
	/** BBQOVN TEST Number. */
	uint8_t indx;

	uint8_t responce;

	uint8_t len;

	uint8_t data[16];

	uint8_t end;
};

typedef void (*test_recv_cb_t)(uint8_t *buf, uint8_t len);

/** @brief Notify heart rate measurement.
 *
 * This will send a GATT notification to all current subscribers.
 *
 *  @param heartrate The heartrate measurement in beats per minute.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int ble_bbqovn_notify_send(uint8_t *buf, uint8_t size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BBQOVN_H_ */
