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
#include "uarthal.h"
#include "leds.h"
#include "uart.h"

// minimum tick time is 1
#define MS2TICK(ms) (ms) > OSENS_SM_TICK_MS ? (ms) / OSENS_SM_TICK_MS : 1

enum {
    OSENS_STATE_INIT = 0,
    OSENS_STATE_SEND_ITF_VER = 1,
    OSENS_STATE_WAIT_ITF_VER_ANS = 2,
    OSENS_STATE_PROC_ITF_VER = 3,
    OSENS_STATE_SEND_BRD_ID = 4,
    OSENS_STATE_WAIT_BRD_ID_ANS = 5,
    OSENS_STATE_PROC_BRD_ID = 6,
    OSENS_STATE_SEND_PT_DESC = 7,
    OSENS_STATE_WAIT_PT_DESC_ANS = 8,
    OSENS_STATE_PROC_PT_DESC = 9,
    OSENS_STATE_BUILD_SCH = 10,
    OSENS_STATE_RUN_SCH = 11,
    OSENS_STATE_SEND_PT_VAL = 12,
    OSENS_STATE_WAIT_PT_VAL_ANS = 13,
    OSENS_STATE_PROC_PT_VAL = 14
};

enum {
	OSENS_STATE_EXEC_OK = 0,
	OSENS_STATE_EXEC_WAIT_OK,
	OSENS_STATE_EXEC_WAIT_STOP,
	OSENS_STATE_EXEC_WAIT_ABORT,
	OSENS_STATE_EXEC_ERROR
};

typedef struct osens_mote_sm_state_s
{
	volatile uint16_t trmout_counter;
	volatile uint16_t trmout;
	volatile uint8_t point_index;
	volatile uint8_t frame_arrived;
	volatile uint8_t state;
	volatile uint8_t retries;
} osens_mote_sm_state_t;

typedef uint8_t (*osens_mote_sm_func_t)(osens_mote_sm_state_t *st);

typedef struct osens_mote_sm_table_s
{
	osens_mote_sm_func_t func;
	uint8_t next_state;
	uint8_t abort_state; // for indicating timeout or end of cyclic operation
	uint8_t error_state;
} osens_mote_sm_table_t;


typedef struct osens_acq_schedule_s
{
	uint8_t num_of_points;
	struct
	{
		uint8_t index;
		uint32_t sampling_time_x250ms;
        uint32_t counter;
	} points[OSENS_MAX_POINTS];

	struct
	{
		uint8_t num_of_points;
		uint8_t index[OSENS_MAX_POINTS];
	} scan;
} osens_acq_schedule_t;

static uint8_t osens_mote_sm_func_build_sch(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_pt_desc_ans(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_req_pt_desc(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_run_sch(osens_mote_sm_state_t *st);

const osens_mote_sm_table_t osens_mote_sm_table[];
const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check osens_datatypes_e order

uint8_t num_rx_bytes;
uint8_t tx_data_len;
uint32_t flagErrorOccurred;

uint8_t frame[OSENS_MAX_FRAME_SIZE];

osens_cmd_req_t cmd;
osens_cmd_res_t ans;

osens_mote_sm_state_t sm_state;
osens_point_ctrl_t sensor_points;
osens_brd_id_t board_info;
osens_acq_schedule_t acquisition_schedule;


//=========================== prototypes =======================================
//=========================== public ==========================================
void sensor_timer(void);
static void osens_mote_tick(void);
static void buBufFlush(void);
void bspLedToggle(uint8_t ui8Leds);
void sensor_sendDone(OpenQueueEntry_t* msg, owerror_t error);

uint8_t osens_mote_init(void)
{

#if (ENABLE_UART0_DAG == 0)
	memset(&sm_state, 0, sizeof(osens_mote_sm_state_t));
	sm_state.state = OSENS_STATE_INIT;

	//buBufFlush();
    opentimers_start(OSENS_SM_TICK_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) osens_mote_tick);

#endif

    return 0;
}


static uint8_t osens_mote_pack_send_frame(osens_cmd_req_t *cmd, uint8_t cmd_size)
{
    uint8_t size;

    size = osens_pack_cmd_req(cmd, frame);

    if (size != cmd_size)
        return OSENS_STATE_EXEC_ERROR;

    if (osens_mote_send_frame(frame, cmd_size) != cmd_size)
    	return OSENS_STATE_EXEC_ERROR;

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_pt_val_ans(osens_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t size;
    uint8_t ans_size;

    point = acquisition_schedule.scan.index[st->point_index];
    ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_READ_POINT_DATA_1 + point))
        return OSENS_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_point_t));

    st->retries = 0;
    st->point_index++;

	leds_debug_toggle();  //LED3

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_req_pt_val(osens_mote_sm_state_t *st)
{
    uint8_t point;

	// end of point reading
    if(st->point_index >= acquisition_schedule.scan.num_of_points)
    	return OSENS_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
	st->retries++;
	if(st->retries > 3)
		return OSENS_STATE_EXEC_ERROR;

	// normal point reading
	point = acquisition_schedule.scan.index[st->point_index];
    cmd.hdr.addr = OSENS_REGMAP_READ_POINT_DATA_1 + point;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);

}

static uint8_t osens_mote_sm_func_run_sch(osens_mote_sm_state_t *st)
{
    uint8_t n;

    acquisition_schedule.scan.num_of_points = 0;

    for (n = 0; n < acquisition_schedule.num_of_points; n++)
    {

    	if(acquisition_schedule.points[n].counter > 0)
    		acquisition_schedule.points[n].counter--;

        if (acquisition_schedule.points[n].counter == 0)
        {
        	acquisition_schedule.scan.index[acquisition_schedule.scan.num_of_points] = n;
        	acquisition_schedule.scan.num_of_points++;
        }

    }

    if(acquisition_schedule.scan.num_of_points > 0)
    {
    	st->point_index = 0;
    	st->retries = 0;
    	return OSENS_STATE_EXEC_WAIT_ABORT;
    }
    else
    	return OSENS_STATE_EXEC_WAIT_OK;
}

/*

{
    osens_mote_read_point(acquisition_schedule.points[n].index);
    acquisition_schedule.points[n].counter = acquisition_schedule.points[n].sampling_time_x250ms;
}
*/

static uint8_t osens_mote_sm_func_build_sch(osens_mote_sm_state_t *st)
{
	uint8_t n, m;

    acquisition_schedule.num_of_points = 0;

    for (n = 0, m = 0; n < board_info.num_of_points; n++)
    {
        if ((sensor_points.points[n].desc.access_rights & OSENS_ACCESS_READ_ONLY) &&
            (sensor_points.points[n].desc.sampling_time_x250ms > 0))
        {
            acquisition_schedule.points[m].index = n;
            acquisition_schedule.points[m].counter = sensor_points.points[n].desc.sampling_time_x250ms;
            acquisition_schedule.points[m].sampling_time_x250ms = sensor_points.points[n].desc.sampling_time_x250ms;

            m++;
            acquisition_schedule.num_of_points++;
        }
    }

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_pt_desc_ans(osens_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 20;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size || (ans.hdr.addr != OSENS_REGMAP_POINT_DESC_1 + st->point_index))
        return OSENS_STATE_EXEC_ERROR;

    memcpy(&sensor_points.points[st->point_index].desc,&ans.payload.point_desc_cmd,sizeof(osens_point_desc_t));

    st->point_index++;
    sensor_points.num_of_points = st->point_index;

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_req_pt_desc(osens_mote_sm_state_t *st)
{
	// end of cyclic point description reading
	board_info.num_of_points = 1;  //teste rff

    if(st->point_index >= board_info.num_of_points)
    	return OSENS_STATE_EXEC_WAIT_ABORT;

    cmd.hdr.addr = OSENS_REGMAP_POINT_DESC_1 + st->point_index;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);
}

static uint8_t osens_mote_sm_func_proc_brd_id_ans(osens_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 28;

    st->point_index = 0;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
    	return OSENS_STATE_EXEC_ERROR;

    memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(osens_brd_id_t));

    if ((board_info.num_of_points == 0) || (board_info.num_of_points > OSENS_MAX_POINTS))
    	return OSENS_STATE_EXEC_ERROR;

    sensor_points.num_of_points = 0;

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_req_brd_id(osens_mote_sm_state_t *st)
{
	cmd.hdr.size = 4;
	cmd.hdr.addr = OSENS_REGMAP_BRD_ID;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);
}


static uint8_t osens_mote_sm_func_proc_itf_ver_ans(osens_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 6;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

	if (size != ans_size)
    	return OSENS_STATE_EXEC_ERROR;

    if ((OSENS_ANS_OK != ans.hdr.status) || (OSENS_LATEST_VERSION != ans.payload.itf_version_cmd.version))
    	return OSENS_STATE_EXEC_ERROR;


    return OSENS_STATE_EXEC_OK;
}


static uint8_t osens_mote_sm_func_wait_ans(osens_mote_sm_state_t *st)
{

	/* TODO!!!! MELHORAR O Tratamento do final de frame
	 * Devido ao problema da UART nao estar reconhecendo o final do frame
	 * a rotina esperar um ciclo de scan (250ms) entao se recebeu algum dado ele diz
	 * que recebeu o frame. Posteriormente sera tratado o frame e verificado se recebeu tudo
	 */
	if ((st->trmout_counter > 0) && (num_rx_bytes > 3))
	{
		st->frame_arrived = true;
		num_rx_bytes = 0;
	}

	if(st->frame_arrived)
	{
		st->frame_arrived = false;
		return OSENS_STATE_EXEC_WAIT_STOP;
	}

	st->trmout_counter++;

	if(st->trmout_counter > st->trmout)
	{
		//se ocorreu timeout e existe bytes no buffer rx considero que recebeu msg
		if (num_rx_bytes > 0)
		{
			return OSENS_STATE_EXEC_WAIT_STOP;
		}
		else
		{
			return OSENS_STATE_EXEC_WAIT_ABORT;
		}
	}

	return OSENS_STATE_EXEC_WAIT_OK;
}


static uint8_t osens_mote_sm_func_req_ver(osens_mote_sm_state_t *st)
{
    cmd.hdr.addr = OSENS_REGMAP_ITF_VERSION;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(500);
    return osens_mote_pack_send_frame(&cmd, 4);
}


static uint8_t osens_mote_sm_func_init(osens_mote_sm_state_t *st)
{
	uint8_t ret = OSENS_STATE_EXEC_OK;

	memset(&cmd, 0, sizeof(cmd));
	memset(&ans, 0, sizeof(ans));
    memset(&sensor_points, 0, sizeof(sensor_points));
	memset(&board_info, 0, sizeof(board_info));
	memset(&acquisition_schedule, 0, sizeof(acquisition_schedule));
	memset(st, 0, sizeof(osens_mote_sm_state_t));

	num_rx_bytes=0;

	return ret;
}

void osens_mote_sm(void)
{
	uint8_t ret;

	#ifdef TRACE_ON
	uint8_t ls = sm_state.state;
	#endif

	ret = osens_mote_sm_table[sm_state.state].func(&sm_state);

	if (flagErrorOccurred)
	{
      //reset uart
	  uart1_clearTxInterrupts();
	  uart1_clearRxInterrupts();      // clear possible pending interrupts
	  uart1_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt

	  flagErrorOccurred = 0;
	}

	switch(ret)
	{
		case OSENS_STATE_EXEC_OK:
		case OSENS_STATE_EXEC_WAIT_STOP:
			sm_state.state = osens_mote_sm_table[sm_state.state].next_state;
			break;
		case OSENS_STATE_EXEC_WAIT_OK:
			// still waiting
			break;
		case OSENS_STATE_EXEC_WAIT_ABORT:
			// wait timeout
			sm_state.state = osens_mote_sm_table[sm_state.state].abort_state;
			break;
		case OSENS_STATE_EXEC_ERROR:
		default:
			sm_state.state = osens_mote_sm_table[sm_state.state].error_state;
			break;
	}

	#ifdef TRACE_ON
    //printf("osens_mote_sm %d -> %d\n",ls,sm_state.state);
	#endif

}

static void osens_mote_tick(void)
{
    scheduler_push_task((task_cbt) osens_mote_sm, TASKPRIO_OSENS_MAIN);
}


const osens_mote_sm_table_t osens_mote_sm_table[] =
{     //{ func,                                   next_state,                      abort_state,                error_state         }
		{ osens_mote_sm_func_init,             OSENS_STATE_SEND_ITF_VER,     OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_INIT
		{ osens_mote_sm_func_req_ver,          OSENS_STATE_WAIT_ITF_VER_ANS, OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_SEND_ITF_VER
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_ITF_VER,     OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_WAIT_ITF_VER_ANS
		{ osens_mote_sm_func_proc_itf_ver_ans, OSENS_STATE_SEND_BRD_ID,      OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_PROC_ITF_VER
		{ osens_mote_sm_func_req_brd_id,       OSENS_STATE_WAIT_BRD_ID_ANS,  OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_SEND_BRD_ID
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_BRD_ID,      OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_WAIT_BRD_ID_ANS
		{ osens_mote_sm_func_proc_brd_id_ans,  OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_PROC_BRD_ID
		{ osens_mote_sm_func_req_pt_desc,      OSENS_STATE_WAIT_PT_DESC_ANS, OSENS_STATE_BUILD_SCH,   OSENS_STATE_INIT }, // OSENS_STATE_SEND_PT_DESC
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_DESC,     OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_WAIT_PT_DESC_ANS
		{ osens_mote_sm_func_pt_desc_ans,      OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_PROC_PT_DESC
		{ osens_mote_sm_func_build_sch,        OSENS_STATE_RUN_SCH,          OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_BUILD_SCH
		{ osens_mote_sm_func_run_sch,          OSENS_STATE_RUN_SCH,          OSENS_STATE_SEND_PT_VAL, OSENS_STATE_INIT }, // OSENS_STATE_RUN_SCH
		{ osens_mote_sm_func_req_pt_val,       OSENS_STATE_WAIT_PT_VAL_ANS,  OSENS_STATE_RUN_SCH,     OSENS_STATE_INIT }, // OSENS_STATE_SEND_PT_VAL
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_VAL,      OSENS_STATE_SEND_PT_VAL, OSENS_STATE_INIT }, // OSENS_STATE_WAIT_PT_VAL_ANS
		{ osens_mote_sm_func_pt_val_ans,       OSENS_STATE_SEND_PT_VAL,      OSENS_STATE_INIT,        OSENS_STATE_INIT }, // OSENS_STATE_PROC_PT_VAL
};

uint8_t osens_init(void)
{
	osens_mote_init();
	return 0;
}

uint8_t osens_get_num_points(void)
{
	if(sm_state.state >= OSENS_STATE_SEND_PT_DESC)
	{
		return board_info.num_of_points;
	}
	else
		return 0;
}

uint8_t osens_get_brd_desc(osens_brd_id_t *brd)
{
	if(sm_state.state >= OSENS_STATE_SEND_PT_DESC)
	{
		memcpy(brd,&board_info,sizeof(osens_brd_id_t));
		return 1;
	}
	else
		return 0;
}

uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc)
{
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index < sensor_points.num_of_points))
	{
		memcpy(desc,&sensor_points.points[index].desc,sizeof(osens_point_desc_t));
		return 1;
	}
	else
		return 0;
}

int8_t osens_get_ptype(uint8_t index)
{
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index < sensor_points.num_of_points))
	{
		return sensor_points.points[index].value.type;
	}
	else
		return -1;
}

uint8_t osens_get_point(uint8_t index, osens_point_t *point)
{
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index < sensor_points.num_of_points))
	{
		memcpy(point,&sensor_points.points[index].value,sizeof(osens_point_t));
		return 1;
	}
	else
		return 0;
}

uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point)
{
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index < sensor_points.num_of_points))
	{
		// preserve the type
		sensor_points.points[index].value.value = point->value;
		return 1;
	}
	else
		return 0;
}

/**************************************************************************//**
* @brief    This function puts up to \e size bytes into the BSP UART TX
*           buffer and starts to transfer data over UART.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is put into the
*           TX buffer only if there is room for all \e size bytes.
*
* @param    frame            is a pointer to the source buffer.
* @param    size            is the number of bytes to transfer.
*
* @return   Returns the number of bytes actually copied to the TX buffer.
******************************************************************************/

uint8_t osens_mote_send_frame(uint8_t *frame, uint8_t size)
{
 	uint16_t ui16Length=size;
    register uint16_t ui16Idx = 0;

    DISABLE_INTERRUPTS();
    while((ui16Idx < ui16Length))
    {
        UARTCharPut(BSP_UART_BASE, frame[ui16Idx++]);
    }
    ENABLE_INTERRUPTS();

    return ((uint8_t) ui16Length);
}

/**************************************************************************//**
* @brief    This function pushes a byte from the UART RX FIFO to the BSP UART
*           RX buffer. The function handles RX buffer wrap-around, but does not
*           handle RX buffer overflow. The function should only be called if
*           there is data in the UART RX FIFO. It modifies volatile variables
*           and should only be called when interrupts are disabled.
*
* @brief    None
******************************************************************************/
static void buBufPushByte(void)
{
    uint32_t value;

    // Push byte from RX FIFO to buffer
    value = UARTCharGetNonBlocking(OSENS_UART_BASE);

    if (num_rx_bytes < OSENS_MAX_FRAME_SIZE)
    	frame[num_rx_bytes] = (uint8_t) value;

    num_rx_bytes++;
    if (num_rx_bytes >= OSENS_MAX_FRAME_SIZE)
        num_rx_bytes = 0;
}


/**************************************************************************//**
* @brief    This function flushes the ringbuffer control structure specified
*           by \e psBuf.
*
* @param    psBuf       is a pointer to a \e tBuBuf ringbuffer structure.
*
* @return   None
******************************************************************************/
static void buBufFlush(void)
{
    // Start of critical section
    bool bIntDisabled = IntMasterDisable();

    num_rx_bytes = 0;

    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


void uart1_isr_private(void)
{
    uint32_t ui32IntBm = UARTIntStatus(OSENS_UART_BASE, 1);
    //uint8_t dummy=0;

    UARTIntClear(OSENS_UART_BASE, (ui32IntBm & 0xF0));

    //ERROR LINE
    if (ui32IntBm & (UART_INT_BE | UART_INT_PE | UART_INT_FE))
	{
    	UARTRxErrorClear(OSENS_UART_BASE);
    	buBufFlush();
    	flagErrorOccurred = 1;
        //while(UARTCharsAvail(OSENS_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_OE))
	{    //OVERRUN
    	UARTRxErrorClear(OSENS_UART_BASE);
        buBufFlush();
    	flagErrorOccurred = 1;
    	//dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //while(UARTCharsAvail(OSENS_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Put received bytes into buffer
        //
        //while(UARTCharsAvail(BSP_UART_BASE) && !BU_IS_BUF_FULL(&sBuBufRx))
        while(UARTCharsAvail(OSENS_UART_BASE))
        {
            buBufPushByte();
        }


        //if (ui32IntBm & UART_INT_RT)
        //{
        //	sm_state.frame_arrived = true;
        //}

    }
    else if(ui32IntBm & UART_INT_TX)
    {
        //while(UARTSpaceAvail(BSP_UART_BASE) && (!BU_IS_BUF_EMPTY(&sBuBufTx)))
        //{
        //    buBufPopByte();
        //}
    }
}

