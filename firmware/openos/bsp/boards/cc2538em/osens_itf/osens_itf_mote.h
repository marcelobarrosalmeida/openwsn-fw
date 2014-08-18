/** @file */

#ifndef __OSENS_MOTE_H__
#define __OSENS_MOTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "osens_itf.h"

#define OSENS_DBG_FRAME  0
#define OSENS_OUTPUT     1
#define OSENS_SM_TICK_MS 250
#define DUMMY_BYTE 0x00


// UART return codes
#define BSP_UART_SUCCESS        0x00
#define BSP_UART_UNCONFIGURED   0x01
#define BSP_UART_NOT_SUPPORTED  0x02
#define BSP_UART_BAUDRATE_ERROR 0x04
#define BSP_UART_ERROR          0x08


#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp uart


#if (USE_SPI_INTERFACE == 0)
//
// This macro gets the byte size of the specified BSP UART ring buffer.
//
#define BU_GET_BUF_SIZE(pRbc)       ((uint32_t)((pRbc)->pui8End) -            \
                                     (uint32_t)((pRbc)->pui8Start))
//
// This macro gets the number of used bytes in the specified BSP UART
// ring buffer.
//
#define BU_GET_USED_SPACE(pRbc)     ((pRbc)->ui16NumBytes)

//

// This macro gets the number of unused bytes in the specified BSP UART
// ring buffer.
//
#define BU_GET_FREE_SPACE(pRbc)     (BU_GET_BUF_SIZE(pRbc) -                  \
                                     ((pRbc)->ui16NumBytes))

//
// This macro checks if the specified BSP UART ring buffer is empty.
//
#define BU_IS_BUF_EMPTY(pRbc)       ((pRbc)->ui16NumBytes == 0)

//
// This macro checks if the specified BSP UART ring buffer is full.
//
#define BU_IS_BUF_FULL(pRbc)        (BU_GET_USED_SPACE(pRbc) ==               \
                                     BU_GET_BUF_SIZE(pRbc))
//
// This macro gets the byte count to tail wrap for the specified BSP UART
// ring buffer.
//
#define BU_BYTES_TO_TAIL_WRAP(pRbc) ((uint32_t)((pRbc)->pui8End) -            \
                                     (uint32_t)((pRbc)->pui8Tail))


#endif




typedef struct
{
    //
    //! Lowest address in buffer
    //
    uint8_t *pui8Start;

    //
    //! Highest address in buffer + 1
    //
    uint8_t *pui8End;

    //
    //! Buffer data input pointer (push)
    //
    volatile uint8_t *pui8Head;

    //
    //! Buffer data output pointer (pop)
    //
    volatile uint8_t *pui8Tail;

    //
    //! Number of stored bytes
    //
    volatile uint16_t ui16NumBytes;
}tBuBuf;

uint8_t osens_mote_init(void);
void bspUartIsrHandler (void);
uint8_t osens_mote_send_frame(uint8_t *frame, uint8_t size);
uint16_t bspUartRxCharsAvail(void);

#ifdef __cplusplus
}
#endif

#endif /* __OSENS_MOTE_H__ */
