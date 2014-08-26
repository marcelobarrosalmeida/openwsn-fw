#include <stdint.h>
#include "openwsn.h"
#include "osens.h"

static osens_brd_id_t board_info;

uint8_t osens_init(void)
{
	memset(&board_info, 0, sizeof(board_info));
	
    strcpy(board_info.model, "PYTHON");
    strcpy(board_info.manufactor, "OWSN");
    board_info.sensor_id = 0xDEADBEEF;
    board_info.hardware_revision = 0x01;
    board_info.num_of_points = 0;
    board_info.capabilities = 0;

    return 1;
}
uint8_t osens_get_brd_desc(osens_brd_id_t *brd)
{
    return 0;
}
uint8_t osens_get_num_points(void)
{
    return 0;
}

uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc)
{
    return 0;
}

int8_t osens_get_ptype( uint8_t index)
{
    return 0;
}

uint8_t osens_get_point(uint8_t index, osens_point_t *point)
{
    return 0;
}

uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point)
{
    return 0;
}

