#include <stdint.h>
#include "openwsn.h"
#include "osens.h"
#include "IEEE802154E.h"

#define NUM_OF_BRD_POINTS 1

typedef struct osens_brd_point_s
{
    osens_point_desc_t  desc;
    osens_point_t value;

} osens_brd_point_t;

static osens_brd_id_t board_info;
static osens_brd_point_t points[NUM_OF_BRD_POINTS];
extern ieee154e_vars_t ieee154e_vars;

uint8_t osens_init(void)
{
    int n;

    memset(&board_info, 0, sizeof(osens_brd_id_t));
	
    for (n = 0; n < NUM_OF_BRD_POINTS; n++)
        memset(&points[n], 0, sizeof(osens_brd_point_t));

    strcpy(board_info.model, "PYTHON");
    strcpy(board_info.manufactor, "OWSN");
    board_info.sensor_id = 0xDEADBEEF;
    board_info.hardware_revision = 0x01;
    board_info.num_of_points = NUM_OF_BRD_POINTS;
    board_info.capabilities = 0;

    strcpy(points[0].desc.name, "ASN");
    points[0].desc.type = OSENS_DT_U64;
    points[0].desc.unit = 0;
    points[0].desc.access_rights = 0;
    points[0].desc.sampling_time_x250ms = 0;

    points[0].value.value.u64 = 0;

    return 1;

}

uint8_t osens_get_brd_desc(osens_brd_id_t *brd)
{
    memcpy(brd, &board_info, sizeof(osens_brd_id_t));
    return 1;
}
uint8_t osens_get_num_points(void)
{
    return board_info.num_of_points;
}

uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc)
{
    if (index < board_info.num_of_points)
    {
        memcpy(desc, &points[index].desc, sizeof(osens_point_desc_t));
        return 1;
    }
    else
        return 0;
}

int8_t osens_get_ptype( uint8_t index)
{
    if (index < board_info.num_of_points)
    {
        return points[index].desc.type;
    }
    else
        return -1;
}

uint8_t osens_get_point(uint8_t index, osens_point_t *point)
{
    if (index < board_info.num_of_points)
    {
        points[index].value.value.u64 = ieee154e_vars.asn.byte4 * 0x100000000 + (ieee154e_vars.asn.bytes2and3 << 16) + ieee154e_vars.asn.bytes0and1;
        memcpy(point, &points[index].value, sizeof(osens_point_t));
        return 1;
    }
    else
        return 0;
}

uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point)
{
    return 0;
}

