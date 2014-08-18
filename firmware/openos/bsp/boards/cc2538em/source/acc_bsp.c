#include "board.h"
#include "stdio.h"
#include <string.h>
#include "hw_ioc.h"             // Access to IOC register defines
#include "hw_ssi.h"             // Access to SSI register defines
#include "hw_sys_ctrl.h"        // Clocking control
#include "hw_memmap.h"
#include "ioc.h"                // Access to driverlib ioc fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "osens.h"
#include "osens_itf.h"
#include "opentimers.h"
#include "scheduler.h"
#include "board.h"
#include "sys_ctrl.h"
#include "osens_itf_mote.h"
#if (USE_SPI_INTERFACE == 0)
//#include "uart.h"
#include "uarthal.h"
#endif
#include "leds.h"
#include "uart.h"
/*
 *  This application is using accelerometer from CC2538em board.
 *  This is only to test a SPI interface
 *  in order to use this, enable the defines
 *   USE_SPI_INTERFACE
 */
//=========================== variables =======================================
#if 0
typedef struct {
   coap_resource_desc_t desc;
   opentimer_id_t  timerId;
} sensor_vars_t;

sensor_vars_t sensor_vars;

const uint8_t sensor_path0[]        = "h";


const osens_mote_sm_table_t osens_mote_sm_table[];

uint8_t frame[OSENS_MAX_FRAME_SIZE];
uint8_t num_rx_bytes;
uint8_t tx_data_len=0;
osens_cmd_req_t cmd;
osens_cmd_res_t ans;
uint32_t flagErrorOccurred=0;
osens_mote_sm_state_t sm_state;
osens_point_ctrl_t sensor_points;
osens_cmd_brd_id_t board_info;
const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check osens_datatypes_e order
osens_acq_schedule_t acquisition_schedule;

//=========================== prototypes =======================================


uint8_t bsp_spi_write_frame(uint8_t *frame, uint8_t size);
uint8_t  bsp_spi_read_frame(uint8_t *pui8Buf, uint8_t size);


static uint8_t osens_mote_sm_func_pt_val_ans(osens_mote_sm_state_t *st)
{
	uint8_t point,ui8Len;
	uint8_t size,ret;
    uint8_t ans_size;
    uint8_t *pui8Buf;
    uint32_t ui32Data;

    point = acquisition_schedule.scan.index[st->point_index];
    ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

#if (USE_SPI_INTERFACE == 1)
#if 0 //(SENSOR_ACCEL == 0)
    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_READ_POINT_DATA_1 + point))
        return OSENS_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_cmd_point_t));

    st->retries = 0;
    st->point_index++;

    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_cmd_point_t));

#else
    //leio dado
    pui8Buf = (uint8_t *) &ans.payload.point_value_cmd;
    ui8Len = ans_size;

    while(ui8Len--)
    {
        //
        // Send dummy byte and read returned data from SPI FIFO
        //
        SSIDataPut(BSP_SPI_SSI_BASE, 0x00);
        SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

        // Store read data to buffer
        //
        *pui8Buf++ = (ui32Data & 0xFF);
    }

    GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, BSP_ACC_CS); // Clear CSn

    //copio para buffer
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_cmd_point_t));
#endif

#else  //treat UART

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_READ_POINT_DATA_1 + point))
        return OSENS_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_cmd_point_t));

    st->retries = 0;
    st->point_index++;

	leds_debug_toggle();  //LED3

#endif

    return OSENS_STATE_EXEC_OK;
}

uint8_t cc2538em_accel_init(void)
{
#if (USE_SPI_INTERFACE == 1)
#if  (SENSOR_ACCEL == 1)
    // Initialize accelerometer
    accInit();
#endif
#endif
}

#endif
