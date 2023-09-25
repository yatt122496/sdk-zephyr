/** @file
 *  @brief BBQOVN Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bbqovn.h>
#include <bluetooth/services/bbqovn_client.h>

#define LOG_LEVEL CONFIG_BT_BBQOVN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bbqovn);


static uint8_t bbqovn_blsc;

extern void bbqovn_notify_remain(void);
static void bbqovn_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("BBQOVN notifications %s", notif_enabled ? "enabled" : "disabled");
	if (notif_enabled) {
		bbqovn_notify_remain();
	}
}

static ssize_t read_blsc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bbqovn_blsc,
				 sizeof(bbqovn_blsc));
}

const static uint8_t sensorDesc[]  				= "BBQOVN-A SENSOR";

enum {
	BBQOVN_SERVER_PS_H = 0,

	BBQOVN_SENSOR_WRITE_CD_H,
	BBQOVN_SENSOR_WRITE_DP_H,

	BBQOVN_SENSOR_NOTIFY_CD_H,
	BBQOVN_SENSOR_NOTIFY_DP_H,
	BBQOVN_SENSOR_CCCB_H,
	BBQOVN_SENSOR_DESC_H,

};

static ssize_t read_bbqovn(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset);
static ssize_t wirte_bbqovn(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset, uint8_t flag);
/* Heart Rate Service Declaration */
BT_GATT_SERVICE_DEFINE(bbqovn_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BBQOVN_UUID_BASE)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(BBQOVN_CHAR_UUID_SENSOR), BT_GATT_CHRC_READ|BT_GATT_CHRC_WRITE_WITHOUT_RESP|BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_bbqovn, wirte_bbqovn, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BBQOVN_NOTIFY_UUID_BASE), BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(bbqovn_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_GATT_CUD, BT_GATT_PERM_READ, read_bbqovn, NULL, NULL),
);

static ssize_t read_bbqovn(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	// LOG_HEXDUMP_INF(buf, len, __FUNCTION__);
	uint8_t responce[20];
	uint8_t responce_len = 0;

	if (attr == &bbqovn_svc.attrs[BBQOVN_SENSOR_DESC_H]) {
		// LOG_INF("BT_GATT_DESCRIPTOR");
		responce_len = sizeof(sensorDesc) - 1;
		memcpy(responce, sensorDesc, responce_len);
	} else if (attr == &bbqovn_svc.attrs[BBQOVN_SENSOR_WRITE_DP_H]) {
		// LOG_INF("BT_GATT_CHARACTERISTIC");
		responce_len = sizeof(bbqovn_blsc);
		memcpy(responce, &bbqovn_blsc, responce_len);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, responce, responce_len);
}

extern void beep_pwm_start(beep_mode_t mode, uint8_t num);

extern void bbqovn_test_cb(uint8_t *buf, uint8_t len);
static ssize_t wirte_bbqovn(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset, uint8_t flag)
{
	LOG_HEXDUMP_INF(buf, len, __FUNCTION__);
	bbqovn_test_cb(buf, len);

	return len;
}

static int bbqovn_init(void)
{
	bbqovn_blsc = 0x01;

	return 0;
}

int ble_bbqovn_notify_send(uint8_t *buf, uint8_t size)
{
	int rc = -ENOTCONN;

	if (buf && size) {
		LOG_HEXDUMP_INF(buf, size, __FUNCTION__);
		rc = bt_gatt_notify(NULL, &bbqovn_svc.attrs[BBQOVN_SENSOR_NOTIFY_DP_H], buf, size);
	}
	LOG_INF("rc=%d", rc);

    return rc;
}

SYS_INIT(bbqovn_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
