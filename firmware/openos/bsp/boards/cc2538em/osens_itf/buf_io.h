/**
@file buf_io.c

Several functions for handling data in buffers. 

Basic functions:

buf_io_[get|put][8|16|32|64|f|d]_[f|t][b|l]_ap[r]

Notation:

8|16|32|64|f|d = data size (f for float, d for double)
[f|t][b|l] = from/to big/little
ap[r] = add pointer [reference] at the end

Check buf_io.c for a proper implementation of these functions for your platform.
*/

#ifndef __BUF_IO__
#define __BUF_IO__

#ifdef __cplusplus
extern "C" {
#endif

/** Pointer size */
#define POINTER_SIZE (sizeof(void *)) 

/** Number of elements in a array */
#define NUM_OF_ELEMENTS_IN_ARRAY(a) (sizeof(a)/sizeof(a[0]))

/** 
  @name next/prev macro functions
  Next/previous aligned address, avoiding data aborts in buffer operations.
  @{
*/
#define __next_aligned_addr32(x)  (((x) + 0x03) & 0xFFFFFFFC)
#define __next_aligned_addr16(x)  (((x) + 0x01) & 0xFFFFFFFE)
#define __prev_aligned_addr32(x)  ((x) & 0xFFFFFFFC)
#define __prev_aligned_addr16(x)  ((x) & 0xFFFFFFFE)
/** @} */

/** 
  @name swap functions
  Swap operations.
  @{
*/

/** 
  Swaps a 16 bits unsigned integer.
  @param ucShort 16 bits unsigned integer to be swapped
  @return 16 bits unsigned integer swapped
  @{
 */
uint16_t buf_io_swap16 (uint16_t usShort);

/** 
  Swaps a 32 bits unsigned integer.
  @param ulLong 32 bits unsigned integer to be swapped
  @return 32 bits unsigned integer swapped
  @{
 */
uint32_t buf_io_swap32 (uint32_t ulLong);

/**
  Swaps a 16 bits unsigned integer inside a buffer.
  @param pucPtr Pointer where 16 bits unsigned integer is stored.
*/
void buf_io_swap16p(uint8_t *pucPtr);

/**
  Swaps a 32 bits unsigned integer inside a buffer.
  @param pucPtr Pointer where 32 bits unsigned integer is stored.
*/
void buf_io_swap32p(uint8_t *pucPtr);

/**
  Swaps 8 bits unsigned integer in a byte.
  @param ucChar 8 bits unsigned integer  to be bit swapped
  @return 8 bits unsigned integer  swapped
*/
uint8_t  buf_io_swap8b (uint8_t ucChar);
/** @} */

/** 
  @name endianess dependent functions (GET)
  @{
*/
uint8_t buf_io_get8_fl       (uint8_t  *buf);
uint8_t buf_io_get8_fb       (uint8_t  *buf);
uint8_t buf_io_get8_fl_apr   (uint8_t **buf);
uint8_t buf_io_get8_fb_apr   (uint8_t **buf);
#define  buf_io_get8_fl_ap(x) buf_io_get8_fl_apr(&x)
#define  buf_io_get8_fb_ap(x) buf_io_get8_fb_apr(&x)

uint16_t buf_io_get16_fl       (uint8_t  *buf);
uint16_t buf_io_get16_fb       (uint8_t  *buf);
uint16_t buf_io_get16_fl_apr   (uint8_t **buf);
uint16_t buf_io_get16_fb_apr   (uint8_t **buf);
#define  buf_io_get16_fl_ap(x) buf_io_get16_fl_apr(&x)
#define  buf_io_get16_fb_ap(x) buf_io_get16_fb_apr(&x)

uint32_t  buf_io_get32_fl       (uint8_t  *buf);
uint32_t  buf_io_get32_fb       (uint8_t  *buf);
uint32_t  buf_io_get32_fl_apr   (uint8_t **buf);
uint32_t  buf_io_get32_fb_apr   (uint8_t **buf);
#define   buf_io_get32_fl_ap(x) buf_io_get32_fl_apr(&x)
#define   buf_io_get32_fb_ap(x) buf_io_get32_fb_apr(&x)

uint64_t buf_io_get64_fl       (uint8_t  *buf);
uint64_t buf_io_get64_fb       (uint8_t  *buf);
uint64_t buf_io_get64_fl_apr   (uint8_t **buf);
uint64_t buf_io_get64_fb_apr   (uint8_t **buf);
#define  buf_io_get64_fl_ap(x) buf_io_get64_fl_apr(&x)
#define  buf_io_get64_fb_ap(x) buf_io_get64_fb_apr(&x)

float   buf_io_getf_fl       (uint8_t *src_ptr);
float   buf_io_getf_fb       (uint8_t *src_ptr);
float   buf_io_getf_fl_apr   (uint8_t **src_ptr);
float   buf_io_getf_fb_apr   (uint8_t **src_ptr);
#define buf_io_getf_fl_ap(x) buf_io_getf_fl_apr(&x)
#define buf_io_getf_fb_ap(x) buf_io_getf_fb_apr(&x)

double  buf_io_getd_fl       (uint8_t *src_ptr);
double  buf_io_getd_fb       (uint8_t *src_ptr);
double  buf_io_getd_fl_apr   (uint8_t **src_ptr);
double  buf_io_getd_fb_apr   (uint8_t **src_ptr);
#define buf_io_getd_fl_ap(x) buf_io_getd_fl_apr(&x)
#define buf_io_getd_fb_ap(x) buf_io_getd_fb_apr(&x)
/** @} */

/** 
  @name endianess dependent functions (PUT)
  @{
*/
void    buf_io_put8_tl         (uint8_t value, uint8_t  *buf);
void    buf_io_put8_tb         (uint8_t value, uint8_t  *buf);
void    buf_io_put8_tl_apr     (uint8_t value, uint8_t **buf);
void    buf_io_put8_tb_apr     (uint8_t value, uint8_t **buf);
#define buf_io_put8_tl_ap(v,x) buf_io_put8_tl_apr(v,&x) 
#define buf_io_put8_tb_ap(v,x) buf_io_put8_tb_apr(v,&x) 

void    buf_io_put16_tl         (uint16_t value, uint8_t  *buf);
void    buf_io_put16_tb         (uint16_t value, uint8_t  *buf);
void    buf_io_put16_tl_apr     (uint16_t value, uint8_t **buf);
void    buf_io_put16_tb_apr     (uint16_t value, uint8_t **buf);
#define buf_io_put16_tl_ap(v,x) buf_io_put16_tl_apr(v,&x) 
#define buf_io_put16_tb_ap(v,x) buf_io_put16_tb_apr(v,&x) 

void    buf_io_put32_tl         (uint32_t value, uint8_t  *buf);
void    buf_io_put32_tb         (uint32_t value, uint8_t  *buf);
void    buf_io_put32_tl_apr     (uint32_t value, uint8_t **buf);
void    buf_io_put32_tb_apr     (uint32_t value, uint8_t **buf);
#define buf_io_put32_tl_ap(v,x) buf_io_put32_tl_apr(v,&x) 
#define buf_io_put32_tb_ap(v,x) buf_io_put32_tb_apr(v,&x) 

void    buf_io_put64_tl         (uint64_t value, uint8_t  *buf);
void    buf_io_put64_tb         (uint64_t value, uint8_t  *buf);
void    buf_io_put64_tl_apr     (uint64_t value, uint8_t **buf);
void    buf_io_put64_tb_apr     (uint64_t value, uint8_t **buf);
#define buf_io_put64_tl_ap(v,x) buf_io_put64_tl_apr(v,&x) 
#define buf_io_put64_tb_ap(v,x) buf_io_put64_tb_apr(v,&x) 

void    buf_io_putf_tl          (float value, uint8_t *buf); 
void    buf_io_putf_tb          (float value, uint8_t *buf); 
void    buf_io_putf_tl_apr      (float value, uint8_t **buf); 
void    buf_io_putf_tb_apr      (float value, uint8_t **buf); 
#define buf_io_putf_tl_ap(v, x) buf_io_putf_tl_apr(v, &x) 
#define buf_io_putf_tb_ap(v, x) buf_io_putf_tb_apr(v, &x) 

void    buf_io_putd_tl          (double value, uint8_t *buf); 
void    buf_io_putd_tb          (double value, uint8_t *buf); 
void    buf_io_putd_tl_apr      (double value, uint8_t **buf); 
void    buf_io_putd_tb_apr      (double value, uint8_t **buf); 
#define buf_io_putd_tl_ap(v, x) buf_io_putd_tl_apr(v, &x) 
#define buf_io_putd_tb_ap(v, x) buf_io_putd_tb_apr(v, &x) 
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __BUF_IO__ */
