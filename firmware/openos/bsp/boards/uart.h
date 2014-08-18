#ifndef __UART_H
#define __UART_H

/**
\addtogroup BSP
\{
\addtogroup uart
\{

\brief Cross-platform declaration "uart" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "stdint.h"
#include "board.h"
 

/* define a UART0 para o DAG ROOT e tambem ja habilita o
 * ponto como dagroot (arquivo openserial flag rffflag)
 * Neste caso a UART1 nao esta disponivel
 */
#if SINK == 1
#define ENABLE_UART0_DAG   1 //TESTE RFF
#define ENABLE_DAG_ROOT_ON_FIRST_TIME  0
#else
#define ENABLE_UART0_DAG   0 //TESTE RFF
#define ENABLE_DAG_ROOT_ON_FIRST_TIME  0
#endif

//SENS_ITF UART1 PINS
#define OSENS_UART_BASE           UART1_BASE
#define OSENS_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART1
#define OSENS_UART_BUS_BASE       GPIO_A_BASE
#define OSENS_UART_RXD_BASE       BSP_UART_BUS_BASE
#define OSENS_UART_RXD            GPIO_PIN_5      //!< PA5
#define OSENS_UART_TXD_BASE       BSP_UART_BUS_BASE
#define OSENS_UART_TXD            GPIO_PIN_4      //!< PA4
#define OSENS_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART1_TXD
#define OSENS_MUX_UART_RXD        IOC_UARTRXD_UART1
#define OSENS_INT_UART            INT_UART1
//=========================== define ==========================================

//=========================== typedef =========================================

typedef enum {
   UART_EVENT_THRES,
   UART_EVENT_OVERFLOW,
} uart_event_t;

typedef void (*uart_tx_cbt)(void);
typedef void (*uart_rx_cbt)(void);

//=========================== variables =======================================

//=========================== prototypes ======================================

void    uart_init(void);
void    uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart_enableInterrupts(void);
void    uart_disableInterrupts(void);
void    uart_clearRxInterrupts(void);
void    uart_clearTxInterrupts(void);
void    uart_writeByte(uint8_t byteToWrite);
uint8_t uart_readByte(void);

//used by SENS_ITF
void    uart1_init(void);
void    uart1_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart1_enableInterrupts(void);
void    uart1_disableInterrupts(void);
void    uart1_clearRxInterrupts(void);
void    uart1_clearTxInterrupts(void);
void    uart1_writeByte(uint8_t byteToWrite);
uint8_t uart1_readByte(void);

void uart1_isr_private(void);

// interrupt handlers
kick_scheduler_t uart_tx_isr(void);
kick_scheduler_t uart_rx_isr(void);

/**
\}
\}
*/

#endif
