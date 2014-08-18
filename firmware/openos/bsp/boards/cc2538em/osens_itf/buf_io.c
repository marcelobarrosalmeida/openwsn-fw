#include <stdint.h>
#include "buf_io.h"

/* --- swap functions ----------------------------  */

uint16_t buf_io_swap16(uint16_t orig_value)
{
    uint16_t value = ((orig_value & 0x00FF) << 8) | 
                     ((orig_value & 0xFF00) >> 8) ;
    return value;
}

uint32_t buf_io_swap32(uint32_t orig_value)
{
    uint32_t value = ((orig_value & 0x000000FF) << 24) | 
                     ((orig_value & 0x0000FF00) << 8 ) |
                     ((orig_value & 0x00FF0000) >> 8 ) |
                     ((orig_value & 0xFF000000) >> 24);
    return value;
}

void buf_io_swap16p(uint8_t *buf)
{
    uint8_t value[2];
    value[1] = *buf++;
    value[0] = *buf++;
    *--buf = value[1];
    *--buf = value[0];
}

void buf_io_swap32p(uint8_t *buf)
{                   
    uint8_t value[4];  
    value[3] = *buf++;            
    value[2] = *buf++;               
    value[1] = *buf++;               
    value[0] = *buf++;   
    *--buf = value[3];               
    *--buf = value[2];               
    *--buf = value[1];               
    *--buf = value[0];               
}

uint8_t buf_io_swap8b(uint8_t orig_value)
{
    uint8_t value = ((orig_value & 0x01) <<  7) |
                    ((orig_value & 0x02) <<  5) |
                    ((orig_value & 0x04) <<  3) |
                    ((orig_value & 0x08) <<  1) |
                    ((orig_value & 0x10) >>  1) |
                    ((orig_value & 0x20) >>  3) |
                    ((orig_value & 0x40) >>  5) |
                    ((orig_value & 0x80) >>  7);
    return value;
}

/* --- 8 bits GET functions ----------------------------  */

uint8_t buf_io_get8_fl(uint8_t *buf)
{
    return buf[0];
}

uint8_t buf_io_get8_fb(uint8_t *buf)
{
    return buf[0];
}

uint8_t buf_io_get8_fl_apr(uint8_t **buf)
{
    uint8_t value = buf_io_get8_fl(*buf);
    *buf += 1;
    return value;
}

uint8_t buf_io_get8_fb_apr(uint8_t **buf)
{
    uint8_t value = buf_io_get8_fb(*buf);
    *buf += 1;
    return value;
}

/* --- 16 bits GET functions ----------------------------  */

uint16_t buf_io_get16_fl(uint8_t *buf)
{
    uint16_t value = buf[0] | (buf[1] << 8);
    return value;
}

uint16_t buf_io_get16_fb(uint8_t *buf)
{
    uint16_t value = buf[1] | (buf[0] << 8);
    return value;
}

uint16_t buf_io_get16_fl_apr(uint8_t **buf)
{
    uint16_t value = buf_io_get16_fl(*buf);
    *buf += 2;
    return value;
}

uint16_t buf_io_get16_fb_apr(uint8_t **buf)
{
    uint16_t value = buf_io_get16_fb(*buf);
    *buf += 2;
    return value;
}

/* --- 32 bits GET functions ----------------------------  */

uint32_t buf_io_get32_fl(uint8_t *buf)
{
    uint32_t value = buf[0] | (buf[1] << 8 ) | (buf[2] << 16) | (buf[3] << 24);
    return value;
}

uint32_t buf_io_get32_fb(uint8_t *buf)
{
    uint32_t value = buf[3] | (buf[2] << 8 ) | (buf[1] << 16) | (buf[0] << 24);
    return value;
}

uint32_t buf_io_get32_fl_apr(uint8_t **buf)
{
    uint32_t value = buf_io_get32_fl(*buf);
    *buf += 4;
    return value;
}

uint32_t buf_io_get32_fb_apr(uint8_t **buf)
{
    uint32_t value = buf_io_get32_fb(*buf);
    *buf += 4;
    return value;
}

/* --- 64 bits GET functions ----------------------------  */

uint64_t buf_io_get64_fl(uint8_t *buf)
{
    uint8_t *p1 =  buf;
    uint8_t *p2 = (buf + 4);
    uint64_t v1 = p1[0] | (p1[1] << 8 ) | (p1[2] << 16) | (p1[3] << 24);
    uint64_t v2 = p2[0] | (p2[1] << 8 ) | (p2[2] << 16) | (p2[3] << 24);
    uint64_t value = (v2 << 32) | v1;
    return value;
}

uint64_t buf_io_get64_fb(uint8_t *buf)
{
    uint8_t *p1 =  buf;
    uint8_t *p2 = (buf + 4);
    uint64_t v1 = p1[3] | (p1[2] << 8 ) | (p1[1] << 16) | (p1[0] << 24);
    uint64_t v2 = p2[3] | (p2[2] << 8 ) | (p2[1] << 16) | (p2[0] << 24);
    uint64_t value = (v1 << 32) | v2;
    return value;
}

uint64_t buf_io_get64_fl_apr(uint8_t **buf)
{
    uint64_t value = buf_io_get64_fl(*buf);
    *buf += 8;
    return value;
}

uint64_t buf_io_get64_fb_apr(uint8_t **buf)
{
    uint64_t value = buf_io_get64_fb(*buf);
    *buf += 8;
    return value;
}

/* --- float GET functions ----------------------------  */

float buf_io_getf_fl(uint8_t *buf)
{
    uint32_t value = buf_io_get32_fl(buf);
    return *((float* )&value);
}

float buf_io_getf_fb(uint8_t *buf)
{
    uint32_t value = buf_io_get32_fb(buf);
    return *((float* )&value);
}

float buf_io_getf_fl_apr(uint8_t **buf)
{
    float value = buf_io_getf_fl(*buf);
    (*buf) += 4;
    return value;
}

float buf_io_getf_fb_apr(uint8_t **buf)
{
    float value = buf_io_getf_fb(*buf);
    (*buf) += 4;
    return value;
}

/* --- double GET functions ----------------------------  */

double buf_io_getd_fl(uint8_t *buf)
{
    uint64_t value = buf_io_get64_fl(buf);
    return *((double* )&value);
}

double buf_io_getd_fb(uint8_t *buf)
{
    uint64_t value = buf_io_get64_fb(buf);
    return *((double* )&value);
}

double buf_io_getd_fl_apr(uint8_t **buf)
{
    double value = buf_io_getd_fl(*buf);
    (*buf) += 8;
    return value;
}

double buf_io_getd_fb_apr(uint8_t **buf)
{
    double ret = buf_io_getd_fb(*buf);
    (*buf) += 8;
    return ret;
}

/* --- 8 bits PUT functions ----------------------------  */

void buf_io_put8_tl(uint8_t value, uint8_t *buf)
{
    buf[0] = value;
}

void buf_io_put8_tb(uint8_t value, uint8_t *buf)
{
    buf[0] = value;
}

void buf_io_put8_tl_apr(uint8_t value, uint8_t **buf)
{
    buf_io_put8_tl(value,*buf);
    *buf += 1;
}

void buf_io_put8_tb_apr(uint8_t value, uint8_t **buf)
{
    buf_io_put8_tb(value,*buf);
    *buf += 1;
}

/* --- 16 bits PUT functions ----------------------------  */

void buf_io_put16_tl(uint16_t value, uint8_t *buf)
{
    buf[0] = (uint8_t)(value     );
    buf[1] = (uint8_t)(value >> 8);
}

void buf_io_put16_tb(uint16_t value, uint8_t *buf)
{
    buf[1] = (uint8_t)(value     );
    buf[0] = (uint8_t)(value >> 8);
}

void buf_io_put16_tl_apr(uint16_t value, uint8_t **buf)
{
    buf_io_put16_tl(value,*buf);
    *buf += 2;
}

void buf_io_put16_tb_apr(uint16_t value, uint8_t **buf)
{
    buf_io_put16_tb(value,*buf);
    *buf += 2;
}

/* --- 32 bits PUT functions ----------------------------  */

void buf_io_put32_tl(uint32_t value, uint8_t *buf)
{
    buf[0] = (uint8_t)(value      );
    buf[1] = (uint8_t)(value >> 8 );
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
}

void buf_io_put32_tb(uint32_t value, uint8_t *buf)
{
    buf[3] = (uint8_t)(value      );
    buf[2] = (uint8_t)(value >> 8 );
    buf[1] = (uint8_t)(value >> 16);
    buf[0] = (uint8_t)(value >> 24);
}

void buf_io_put32_tl_apr(uint32_t value, uint8_t **buf)
{
    buf_io_put32_tl(value,*buf);
    *buf += 4;
}

void buf_io_put32_tb_apr(uint32_t value, uint8_t **buf)
{
    buf_io_put32_tb(value,*buf);
    *buf += 4;
}

/* --- 64 bits PUT functions ----------------------------  */

void buf_io_put64_tl(uint64_t value, uint8_t *buf)
{
    buf[0] = (uint8_t)(value      );
    buf[1] = (uint8_t)(value >> 8 );
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
    buf[4] = (uint8_t)(value >> 32);
    buf[5] = (uint8_t)(value >> 40);
    buf[6] = (uint8_t)(value >> 48);
    buf[7] = (uint8_t)(value >> 56);
}

void buf_io_put64_tb(uint64_t value, uint8_t *buf)
{
    buf[7] = (uint8_t)(value      );
    buf[6] = (uint8_t)(value >> 8 );
    buf[5] = (uint8_t)(value >> 16);
    buf[4] = (uint8_t)(value >> 24);
    buf[3] = (uint8_t)(value >> 32);
    buf[2] = (uint8_t)(value >> 40);
    buf[1] = (uint8_t)(value >> 48);
    buf[0] = (uint8_t)(value >> 56);
}

void buf_io_put64_tl_apr(uint64_t value, uint8_t **buf)
{
    buf_io_put64_tl(value,*buf);
    *buf += 8;
}

void buf_io_put64_tb_apr(uint64_t value, uint8_t **buf)
{
    buf_io_put64_tb(value,*buf);
    *buf += 8;
}

/* --- float PUT functions ----------------------------  */

void buf_io_putf_tl(float value, uint8_t *buf)
{
    buf_io_put32_tl(*((uint32_t*) &value),buf);
}

void buf_io_putf_tb(float value, uint8_t *buf)
{
    buf_io_put32_tb(*((uint32_t*) &value),buf);
}

void buf_io_putf_tl_apr(float value, uint8_t **buf)
{
    buf_io_putf_tl(value,*buf);
    *buf += 4;
}

void buf_io_putf_tb_apr(float value, uint8_t **buf)
{
    buf_io_putf_tb(value,*buf);
    *buf += 4;
}

/* --- double PUT functions ----------------------------  */

void buf_io_putd_tl(double value, uint8_t *buf)
{
    buf_io_put64_tl(*((uint64_t*) &value),buf);
}

void buf_io_putd_tb(double value, uint8_t *buf)
{
    buf_io_put64_tb(*((uint64_t*) &value),buf);
}

void buf_io_putd_tl_apr(double value, uint8_t **buf)
{
    buf_io_putd_tl(value,*buf);
    *buf += 8;
}

void buf_io_putd_tb_apr(double value, uint8_t **buf)
{
    buf_io_putd_tb(value,*buf);
    *buf += 8;
}
