/**
@file osens.h
@brief Open sensors driver interface

@author Marcelo Barros de Almeida <marcelobarrosalmeida@gmail.com>
*/

#ifndef __OSENS_H__
#define __OSENS_H__

#define OSENS_LATEST_VERSION     0
#define OSENS_MODEL_NAME_SIZE    8
#define OSENS_MANUF_NAME_SIZE    8
#define OSENS_POINT_NAME_SIZE    8
#define OSENS_MAX_POINTS        32

enum osens_access_rights_e
{
	OSENS_ACCESS_READ_ONLY = 0x01,
	OSENS_ACCESS_WRITE_ONLY = 0x02,
	OSENS_ACCESS_READ_WRITE = 0x03,
};

enum osens_sensor_capabilities_e
{
	OSENS_CAPABILITIES_BATTERY = 0x01,
	OSENS_CAPABILITIES_DISPLAY = 0x02,
	OSENS_CAPABILITIES_WPAN_STATUS = 0x04,
	OSENS_CAPABILITIES_BATTERY_STATUS = 0x08,
};

/** Sensor interface standard datatypes */
enum osens_datatypes_e
{
	OSENS_DT_U8     = 0x00, /**< 8 bits unsigned */
	OSENS_DT_S8     = 0x01, /**< 8 bits signed */
	OSENS_DT_U16    = 0x02, /**< 16 bits unsigned */
	OSENS_DT_S16    = 0x03, /**< 16 bits signed */
	OSENS_DT_U32    = 0x04, /**< 32 bits unsigned */
	OSENS_DT_S32    = 0x05, /**< 32 bits signed */
	OSENS_DT_U64    = 0x06, /**< 64 bits unsigned */
	OSENS_DT_S64    = 0x07, /**< 64 bits signed */
	OSENS_DT_FLOAT  = 0x08, /**< IEEE 754 single precision */
	OSENS_DT_DOUBLE = 0x09, /**< IEEE 754 double precision */
};

union osens_point_data_u
{
	uint8_t  u8;
	int8_t   s8;
	uint16_t u16;
	int16_t  s16;
	uint32_t u32;
	int32_t  s32;
	uint64_t u64;
	int64_t  s64;
	float    fp32;
	double   fp64;
};

typedef struct osens_brd_id_s
{
	uint8_t model[OSENS_MODEL_NAME_SIZE];
	uint8_t manufactor[OSENS_MANUF_NAME_SIZE];
	uint32_t sensor_id;
	uint8_t hardware_revision;
	uint8_t num_of_points;
	uint8_t capabilities;
} osens_brd_id_t;


typedef struct osens_point_desc_s
{
	uint8_t name[OSENS_POINT_NAME_SIZE];
	uint8_t type;
	uint8_t unit;
	uint8_t access_rights;
	uint32_t sampling_time_x250ms;
} osens_point_desc_t;

typedef struct osens_point_s
{
	union osens_point_data_u value;
    uint8_t type;
} osens_point_t;


uint8_t osens_init(void);
uint8_t osens_get_brd_desc(osens_brd_id_t *brd);
uint8_t osens_get_num_points(void);
uint8_t osens_get_point(uint8_t index, osens_point_t *point);
uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc);
int8_t osens_get_ptype(uint8_t index);
uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point);

#endif /* __OSENS_H__ */

