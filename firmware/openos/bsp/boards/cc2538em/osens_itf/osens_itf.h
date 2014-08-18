/** @file */

#ifndef __OSENS_ITF_H__
#define __OSENS_ITF_H__

#ifdef __cplusplus
extern "C" {
#endif

#define OSENS_MAX_FRAME_SIZE   128
#define OSENS_DSP_MSG_MAX_SIZE  24
#define OSENS_SERVER_ADDR_SIZE  16

/** Sensor interface register map */
enum osens_register_map_e 
{
	OSENS_REGMAP_ITF_VERSION      = 0x00, /**< Sensor Board Interface Version */
	OSENS_REGMAP_BRD_ID           = 0x01, /**< Sensor Board Identification */
	OSENS_REGMAP_BRD_STATUS       = 0x02, /**< Sensor Board Status */
	OSENS_REGMAP_BRD_CMD          = 0x03, /**< Sensor Board Command */
	OSENS_REGMAP_READ_BAT_STATUS  = 0x04, /**< Read battery status */
    OSENS_REGMAP_WRITE_BAT_STATUS = 0x05, /**< Write battery status */
	OSENS_REGMAP_READ_BAT_CHARGE  = 0x06, /**< Read battery charge(0 - 100 % ) */
    OSENS_REGMAP_WRITE_BAT_CHARGE = 0x07, /**< Write Battery charge(0 - 100 % ) */
	OSENS_REGMAP_WPAN_STATUS      = 0x08, /**< Wireless network status */
	OSENS_REGMAP_WPAN_STRENGTH    = 0x09, /**< Wireless network strength(RSSI, 0 to 100 % ) */
	OSENS_REGMAP_DSP_WRITE        = 0x0A, /**< Write display (when display is available) */
	OSENS_REGMAP_SVR_MAIN_ADDR    = 0x0B, /**< Main server address(IPv6) */
	OSENS_REGMAP_SVR_SEC_ADDR     = 0x0C, /**< Secondary server address(IPv6) */

	/*0x0D to 0x0F - Reserved(Should not be answered) */

	OSENS_REGMAP_POINT_DESC_1  = 0x10, /**< Sensor Point Description 1 */
	OSENS_REGMAP_POINT_DESC_2  = 0x11, /**< Sensor Point Description 2 */
	OSENS_REGMAP_POINT_DESC_3  = 0x12, /**< Sensor Point Description 3 */
	OSENS_REGMAP_POINT_DESC_4  = 0x13, /**< Sensor Point Description 4 */
	OSENS_REGMAP_POINT_DESC_5  = 0x14, /**< Sensor Point Description 5 */
	OSENS_REGMAP_POINT_DESC_6  = 0x15, /**< Sensor Point Description 6 */
	OSENS_REGMAP_POINT_DESC_7  = 0x16, /**< Sensor Point Description 7 */
	OSENS_REGMAP_POINT_DESC_8  = 0x17, /**< Sensor Point Description 8 */
	OSENS_REGMAP_POINT_DESC_9  = 0x18, /**< Sensor Point Description 9 */
	OSENS_REGMAP_POINT_DESC_10 = 0x19, /**< Sensor Point Description 10 */
	OSENS_REGMAP_POINT_DESC_11 = 0x1A, /**< Sensor Point Description 11 */
	OSENS_REGMAP_POINT_DESC_12 = 0x1B, /**< Sensor Point Description 12 */
	OSENS_REGMAP_POINT_DESC_13 = 0x1C, /**< Sensor Point Description 13 */
	OSENS_REGMAP_POINT_DESC_14 = 0x1D, /**< Sensor Point Description 14 */
	OSENS_REGMAP_POINT_DESC_15 = 0x1E, /**< Sensor Point Description 15 */
	OSENS_REGMAP_POINT_DESC_16 = 0x1F, /**< Sensor Point Description 16 */
	OSENS_REGMAP_POINT_DESC_17 = 0x20, /**< Sensor Point Description 17 */
	OSENS_REGMAP_POINT_DESC_18 = 0x21, /**< Sensor Point Description 18 */
	OSENS_REGMAP_POINT_DESC_19 = 0x22, /**< Sensor Point Description 19 */
	OSENS_REGMAP_POINT_DESC_20 = 0x23, /**< Sensor Point Description 20 */
	OSENS_REGMAP_POINT_DESC_21 = 0x24, /**< Sensor Point Description 21 */
	OSENS_REGMAP_POINT_DESC_22 = 0x25, /**< Sensor Point Description 22 */
	OSENS_REGMAP_POINT_DESC_23 = 0x26, /**< Sensor Point Description 23 */
	OSENS_REGMAP_POINT_DESC_24 = 0x27, /**< Sensor Point Description 24 */
	OSENS_REGMAP_POINT_DESC_25 = 0x28, /**< Sensor Point Description 25 */
	OSENS_REGMAP_POINT_DESC_26 = 0x29, /**< Sensor Point Description 26 */
	OSENS_REGMAP_POINT_DESC_27 = 0x2A, /**< Sensor Point Description 27 */
	OSENS_REGMAP_POINT_DESC_28 = 0x2B, /**< Sensor Point Description 28 */
	OSENS_REGMAP_POINT_DESC_29 = 0x2C, /**< Sensor Point Description 29 */
	OSENS_REGMAP_POINT_DESC_30 = 0x2D, /**< Sensor Point Description 30 */
	OSENS_REGMAP_POINT_DESC_31 = 0x2E, /**< Sensor Point Description 31 */
	OSENS_REGMAP_POINT_DESC_32 = 0x2F, /**< Sensor Point Description 32 */

	OSENS_REGMAP_READ_POINT_DATA_1 = 0x30, /**< Read Sensor Point Data 1 */
	OSENS_REGMAP_READ_POINT_DATA_2 = 0x31, /**< Read Sensor Point Data 2 */
	OSENS_REGMAP_READ_POINT_DATA_3 = 0x32, /**< Read Sensor Point Data 3 */
	OSENS_REGMAP_READ_POINT_DATA_4 = 0x33, /**< Read Sensor Point Data 4 */
	OSENS_REGMAP_READ_POINT_DATA_5 = 0x34, /**< Read Sensor Point Data 5 */
	OSENS_REGMAP_READ_POINT_DATA_6 = 0x35, /**< Read Sensor Point Data 6 */
	OSENS_REGMAP_READ_POINT_DATA_7 = 0x36, /**< Read Sensor Point Data 7 */
	OSENS_REGMAP_READ_POINT_DATA_8 = 0x37, /**< Read Sensor Point Data 8 */
	OSENS_REGMAP_READ_POINT_DATA_9 = 0x38, /**< Read Sensor Point Data 9 */
	OSENS_REGMAP_READ_POINT_DATA_10 = 0x39, /**< Read Sensor Point Data 10 */
	OSENS_REGMAP_READ_POINT_DATA_11 = 0x3A, /**< Read Sensor Point Data 11 */
	OSENS_REGMAP_READ_POINT_DATA_12 = 0x3B, /**< Read Sensor Point Data 12 */
	OSENS_REGMAP_READ_POINT_DATA_13 = 0x3C, /**< Read Sensor Point Data 13 */
	OSENS_REGMAP_READ_POINT_DATA_14 = 0x3D, /**< Read Sensor Point Data 14 */
	OSENS_REGMAP_READ_POINT_DATA_15 = 0x3E, /**< Read Sensor Point Data 15 */
	OSENS_REGMAP_READ_POINT_DATA_16 = 0x3F, /**< Read Sensor Point Data 16 */
	OSENS_REGMAP_READ_POINT_DATA_17 = 0x40, /**< Read Sensor Point Data 17 */
	OSENS_REGMAP_READ_POINT_DATA_18 = 0x41, /**< Read Sensor Point Data 18 */
	OSENS_REGMAP_READ_POINT_DATA_19 = 0x42, /**< Read Sensor Point Data 19 */
	OSENS_REGMAP_READ_POINT_DATA_20 = 0x43, /**< Read Sensor Point Data 20 */
	OSENS_REGMAP_READ_POINT_DATA_21 = 0x44, /**< Read Sensor Point Data 21 */
	OSENS_REGMAP_READ_POINT_DATA_22 = 0x45, /**< Read Sensor Point Data 22 */
	OSENS_REGMAP_READ_POINT_DATA_23 = 0x46, /**< Read Sensor Point Data 23 */
	OSENS_REGMAP_READ_POINT_DATA_24 = 0x47, /**< Read Sensor Point Data 24 */
	OSENS_REGMAP_READ_POINT_DATA_25 = 0x48, /**< Read Sensor Point Data 25 */
	OSENS_REGMAP_READ_POINT_DATA_26 = 0x49, /**< Read Sensor Point Data 26 */
	OSENS_REGMAP_READ_POINT_DATA_27 = 0x4A, /**< Read Sensor Point Data 27 */
	OSENS_REGMAP_READ_POINT_DATA_28 = 0x4B, /**< Read Sensor Point Data 28 */
	OSENS_REGMAP_READ_POINT_DATA_29 = 0x4C, /**< Read Sensor Point Data 29 */
	OSENS_REGMAP_READ_POINT_DATA_30 = 0x4D, /**< Read Sensor Point Data 30 */
	OSENS_REGMAP_READ_POINT_DATA_31 = 0x4E, /**< Read Sensor Point Data 31 */
	OSENS_REGMAP_READ_POINT_DATA_32 = 0x4F, /**< Read Sensor Point Data 32 */

	OSENS_REGMAP_WRITE_POINT_DATA_1 = 0x50, /**< Write Sensor Point Data 1 */
	OSENS_REGMAP_WRITE_POINT_DATA_2 = 0x51, /**< Write Sensor Point Data 2 */
	OSENS_REGMAP_WRITE_POINT_DATA_3 = 0x52, /**< Write Sensor Point Data 3 */
	OSENS_REGMAP_WRITE_POINT_DATA_4 = 0x53, /**< Write Sensor Point Data 4 */
	OSENS_REGMAP_WRITE_POINT_DATA_5 = 0x54, /**< Write Sensor Point Data 5 */
	OSENS_REGMAP_WRITE_POINT_DATA_6 = 0x55, /**< Write Sensor Point Data 6 */
	OSENS_REGMAP_WRITE_POINT_DATA_7 = 0x56, /**< Write Sensor Point Data 7 */
	OSENS_REGMAP_WRITE_POINT_DATA_8 = 0x57, /**< Write Sensor Point Data 8 */
	OSENS_REGMAP_WRITE_POINT_DATA_9 = 0x58, /**< Write Sensor Point Data 9 */
	OSENS_REGMAP_WRITE_POINT_DATA_10 = 0x59, /**< Write Sensor Point Data 10 */
	OSENS_REGMAP_WRITE_POINT_DATA_11 = 0x5A, /**< Write Sensor Point Data 11 */
	OSENS_REGMAP_WRITE_POINT_DATA_12 = 0x5B, /**< Write Sensor Point Data 12 */
	OSENS_REGMAP_WRITE_POINT_DATA_13 = 0x5C, /**< Write Sensor Point Data 13 */
	OSENS_REGMAP_WRITE_POINT_DATA_14 = 0x5D, /**< Write Sensor Point Data 14 */
	OSENS_REGMAP_WRITE_POINT_DATA_15 = 0x5E, /**< Write Sensor Point Data 15 */
	OSENS_REGMAP_WRITE_POINT_DATA_16 = 0x5F, /**< Write Sensor Point Data 16 */
	OSENS_REGMAP_WRITE_POINT_DATA_17 = 0x60, /**< Write Sensor Point Data 17 */
	OSENS_REGMAP_WRITE_POINT_DATA_18 = 0x61, /**< Write Sensor Point Data 18 */
	OSENS_REGMAP_WRITE_POINT_DATA_19 = 0x62, /**< Write Sensor Point Data 19 */
	OSENS_REGMAP_WRITE_POINT_DATA_20 = 0x63, /**< Write Sensor Point Data 20 */
	OSENS_REGMAP_WRITE_POINT_DATA_21 = 0x64, /**< Write Sensor Point Data 21 */
	OSENS_REGMAP_WRITE_POINT_DATA_22 = 0x65, /**< Write Sensor Point Data 22 */
	OSENS_REGMAP_WRITE_POINT_DATA_23 = 0x66, /**< Write Sensor Point Data 23 */
	OSENS_REGMAP_WRITE_POINT_DATA_24 = 0x67, /**< Write Sensor Point Data 24 */
	OSENS_REGMAP_WRITE_POINT_DATA_25 = 0x68, /**< Write Sensor Point Data 25 */
	OSENS_REGMAP_WRITE_POINT_DATA_26 = 0x69, /**< Write Sensor Point Data 26 */
	OSENS_REGMAP_WRITE_POINT_DATA_27 = 0x6A, /**< Write Sensor Point Data 27 */
	OSENS_REGMAP_WRITE_POINT_DATA_28 = 0x6B, /**< Write Sensor Point Data 28 */
	OSENS_REGMAP_WRITE_POINT_DATA_29 = 0x6C, /**< Write Sensor Point Data 29 */
	OSENS_REGMAP_WRITE_POINT_DATA_30 = 0x6D, /**< Write Sensor Point Data 30 */
	OSENS_REGMAP_WRITE_POINT_DATA_31 = 0x6E, /**< Write Sensor Point Data 31 */
	OSENS_REGMAP_WRITE_POINT_DATA_32 = 0x6F, /**< Write Sensor Point Data 32 */

	/* 0x70 to 0xFF - Reserved */
};

enum osens_sensor_status_e
{
	OSENS_SENSOR_STATUS_OK = 0,
	OSENS_SENSOR_STATUS_POINT_1 = 1,
	OSENS_SENSOR_STATUS_POINT_2 = 2,
	OSENS_SENSOR_STATUS_POINT_3 = 3,
	OSENS_SENSOR_STATUS_POINT_4 = 4,
	OSENS_SENSOR_STATUS_POINT_5 = 5,
	OSENS_SENSOR_STATUS_POINT_6 = 6,
	OSENS_SENSOR_STATUS_POINT_7 = 7,
	OSENS_SENSOR_STATUS_POINT_8 = 8,
	OSENS_SENSOR_STATUS_POINT_9 = 9,
	OSENS_SENSOR_STATUS_POINT_10 = 10,
	OSENS_SENSOR_STATUS_POINT_11 = 11,
	OSENS_SENSOR_STATUS_POINT_12 = 12,
	OSENS_SENSOR_STATUS_POINT_13 = 13,
	OSENS_SENSOR_STATUS_POINT_14 = 14,
	OSENS_SENSOR_STATUS_POINT_15 = 15,
	OSENS_SENSOR_STATUS_POINT_16 = 16,
	OSENS_SENSOR_STATUS_POINT_17 = 17,
	OSENS_SENSOR_STATUS_POINT_18 = 18,
	OSENS_SENSOR_STATUS_POINT_19 = 19,
	OSENS_SENSOR_STATUS_POINT_20 = 20,
	OSENS_SENSOR_STATUS_POINT_21 = 21,
	OSENS_SENSOR_STATUS_POINT_22 = 22,
	OSENS_SENSOR_STATUS_POINT_23 = 23,
	OSENS_SENSOR_STATUS_POINT_24 = 24,
	OSENS_SENSOR_STATUS_POINT_25 = 25,
	OSENS_SENSOR_STATUS_POINT_26 = 26,
	OSENS_SENSOR_STATUS_POINT_27 = 27,
	OSENS_SENSOR_STATUS_POINT_28 = 28,
	OSENS_SENSOR_STATUS_POINT_29 = 29,
	OSENS_SENSOR_STATUS_POINT_30 = 30,
	OSENS_SENSOR_STATUS_POINT_31 = 31,
	OSENS_SENSOR_STATUS_POINT_32 = 32,
	/* 33 to 127 - reserved */
	OSENS_SENSOR_STATUS_GENERAL_SENSOR_FAILURE = 128,
	/* 129 to 255 - reserved */
};

enum osens_sensor_cmds_e
{
	OSENS_SENSOR_CMD_RESET = 0,
};

enum osens_ans_status_e
{
	OSENS_ANS_OK = 0,
	OSENS_ANS_ERROR = 1,
    OSENS_ANS_CRC_ERROR = 2,
	OSENS_ANS_READY_ONLY = 3,
	OSENS_ANS_WRITE_ONLY = 4,
    OSENS_ANS_REGISTER_NOT_IMPLEMENTED = 5,
};

enum osens_bat_status_e
{
	OSENS_BAT_STATUS_CHARGED = 0x00,
	OSENS_BAT_STATUS_CHARGING = 0x01,
	OSENS_BAT_STATUS_DISCHARGING = 0x02,
	OSENS_BAT_STATUS_FAILURE = 0x03,
};

enum osens_wpan_status_e
{
	OSENS_WPAN_STATUS_CONNECTED = 0x00,
	OSENS_WPAN_STATUS_DISCONNECTED = 0x01,
	OSENS_WPAN_STATUS_CONNECTING = 0x02,
};

typedef struct osens_bat_status_s
{
	uint8_t status;
} osens_bat_status_t;

typedef struct osens_bat_charge_s
{
	uint8_t charge;
} osens_bat_charge_t;

typedef struct osens_command_s
{
	uint8_t cmd;
} osens_command_t;

typedef struct osens_command_res_s
{
	uint8_t status;
} osens_command_res_t;

typedef struct osens_wpan_status_s
{
	uint8_t status;
} osens_wpan_status_t;

typedef struct osens_wpan_strenght_s
{
	uint8_t strenght;
} osens_wpan_strenght_t;

typedef struct osens_write_display_s
{
	uint8_t line;
	uint8_t msg[OSENS_DSP_MSG_MAX_SIZE];
} osens_write_display_t;

typedef struct osens_svr_addr_s
{
	uint8_t addr[OSENS_SERVER_ADDR_SIZE];
} osens_svr_addr_t;

typedef struct osens_itf_version_s
{
	uint8_t version;
} osens_itf_version_t;

typedef struct osens_brd_status_s
{
	uint8_t status;
} osens_brd_status_t;

typedef struct osens_point_ctrl_s
{
	uint8_t num_of_points;
	struct {
		osens_point_desc_t desc;
		osens_point_t value;
	} points[OSENS_MAX_POINTS];
} osens_point_ctrl_t;

union osens_cmds_u
{
	osens_bat_status_t bat_status_cmd;
	osens_bat_charge_t bat_charge_cmd;
    osens_command_t command_cmd;
    osens_command_res_t command_res_cmd;
	osens_wpan_status_t wpan_status_cmd;
	osens_wpan_strenght_t wpan_strength_cmd;
	osens_write_display_t write_display_cmd;
	osens_svr_addr_t svr_addr_cmd;
	osens_itf_version_t itf_version_cmd;
	osens_brd_id_t brd_id_cmd;
	osens_brd_status_t brd_status_cmd;
	osens_point_desc_t point_desc_cmd;
	osens_point_t point_value_cmd;
};

typedef struct osens_cmd_req_hdr_s
{
	uint8_t size;
	uint8_t addr;
} osens_cmd_req_hdr_t;

typedef struct osens_cmd_res_hdr_s
{
	uint8_t size;
	uint8_t status;
    uint8_t addr;
} osens_cmd_res_hdr_t;

typedef struct osens_cmd_req_s
{
	osens_cmd_req_hdr_t hdr;
	union osens_cmds_u payload;
	uint16_t crc;
} osens_cmd_req_t;

typedef struct osens_cmd_res_s
{
	osens_cmd_res_hdr_t hdr;
	union osens_cmds_u payload;
	uint16_t crc;
} osens_cmd_res_t;


//uint8_t osens_send_cmd(osens_cmd_req_t * cmd, osens_cmd_res_t * ans);
//int osens_send_cmd_async(const osens_cmd_req_t * const cmd, const osens_cmd_res_t * ans);

//uint8_t osens_mote_init(void);
//uint8_t osens_sensor_init(void);
//void osens_mote_main(void);

uint8_t osens_unpack_point_value(osens_point_t *point, uint8_t *buf);
uint8_t osens_pack_point_value(const osens_point_t *point, uint8_t *buf);

uint8_t osens_unpack_cmd_res(osens_cmd_res_t *cmd, uint8_t *frame, uint8_t frame_size);
uint8_t osens_unpack_cmd_req(osens_cmd_req_t *cmd, uint8_t *frame, uint8_t frame_size);
uint8_t osens_pack_cmd_res  (osens_cmd_res_t *cmd, uint8_t *frame);
uint8_t osens_pack_cmd_req  (osens_cmd_req_t *cmd, uint8_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __OSENS_ITF_H__ */
