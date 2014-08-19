#include <stdint.h>
#include "openwsn.h"
#include "osens.h"

uint8_t osens_init(void)
{
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

