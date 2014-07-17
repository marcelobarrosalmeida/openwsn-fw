#include <string.h>
#include "openwsn.h"
#include "sens_itf.h"
#include "opentimers.h"
#include "scheduler.h"

#define SENS_ITF_DBG_FRAME 0
#define SENS_ITF_OUTPUT    1
#define SENS_ITF_SM_TICK_MS 250
// minimum tick time is 1
#define MS2TICK(ms) (ms) > SENS_ITF_SM_TICK_MS ? (ms) / SENS_ITF_SM_TICK_MS : 1

enum {
    SENS_ITF_STATE_INIT = 0,
    SENS_ITF_STATE_SEND_ITF_VER = 1,
    SENS_ITF_STATE_WAIT_ITF_VER_ANS = 2,
    SENS_ITF_STATE_PROC_ITF_VER = 3,
    SENS_ITF_STATE_SEND_BRD_ID = 4,
    SENS_ITF_STATE_WAIT_BRD_ID_ANS = 5,
    SENS_ITF_STATE_PROC_BRD_ID = 6,
    SENS_ITF_STATE_SEND_PT_DESC = 7,
    SENS_ITF_STATE_WAIT_PT_DESC_ANS = 8,
    SENS_ITF_STATE_PROC_PT_DESC = 9,
    SENS_ITF_STATE_BUILD_SCH = 10,
    SENS_ITF_STATE_RUN_SCH = 11,
    SENS_ITF_STATE_SEND_PT_VAL = 12,
    SENS_ITF_STATE_WAIT_PT_VAL_ANS = 13,
    SENS_ITF_STATE_PROC_PT_VAL = 14
};

enum {
	SENS_ITF_STATE_EXEC_OK = 0,
	SENS_ITF_STATE_EXEC_WAIT_OK,
	SENS_ITF_STATE_EXEC_WAIT_STOP,
	SENS_ITF_STATE_EXEC_WAIT_ABORT,
	SENS_ITF_STATE_EXEC_ERROR
};

typedef struct sens_itf_mote_sm_state_s
{
	volatile uint16_t trmout_counter;
	volatile uint16_t trmout;
	volatile uint8_t point_index;
	volatile uint8_t frame_arrived;
	volatile uint8_t state;
	volatile uint8_t retries;
} sens_itf_mote_sm_state_t;

typedef uint8_t (*sens_itf_mote_sm_func_t)(sens_itf_mote_sm_state_t *st);

typedef struct sens_itf_mote_sm_table_s
{
	sens_itf_mote_sm_func_t func;
	uint8_t next_state;
	uint8_t abort_state; // for indicating timeout or end of cyclic operation
	uint8_t error_state;
} sens_itf_mote_sm_table_t;


typedef struct sens_itf_acq_schedule_s
{
	uint8_t num_of_points;
	struct 
	{
		uint8_t index;
		uint32_t sampling_time_x250ms;
        uint32_t counter;
	} points[SENS_ITF_MAX_POINTS];
	struct
	{
		uint8_t num_of_points;
		uint8_t index[SENS_ITF_MAX_POINTS];
	} scan;
} sens_itf_acq_schedule_t;


static sens_itf_mote_sm_state_t sm_state;
static sens_itf_point_ctrl_t sensor_points;
static sens_itf_cmd_brd_id_t board_info;
static const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check sens_itf_datatypes_e order
static sens_itf_acq_schedule_t acquisition_schedule;
static uint8_t frame[SENS_ITF_MAX_FRAME_SIZE];
static sens_itf_cmd_req_t cmd;
static sens_itf_cmd_res_t ans;

#if 0
/*
// Serial or SPI interrupt, called when a new byte is received
static void sens_itf_sensor_rx_byte(void)
{
    uint8_t value;

    // DISABLE INTERRUPTS
    if (frame_timeout)
    {   
        // empty register
        return;
}

	value = 0;// PUT CHANNEL HERE (uint8_t) pcSerial.getc();

    if (num_rx_bytes < SENS_ITF_MAX_FRAME_SIZE)
        rx_frame[num_rx_bytes] = value;

    num_rx_bytes++;
    if (num_rx_bytes >= SENS_ITF_MAX_FRAME_SIZE)
        num_rx_bytes = 0;

    // ENABLE INTERRUPTS
}
*/
#endif

extern sens_itf_mote_sm_table_t sens_itf_mote_sm_table[];

static uint8_t sens_itf_mote_send_frame(uint8_t *frame, uint8_t size)
{
    //int16_t sent;
   // return (sent < 0 ? 0 : (uint8_t) sent); // CHECK AGAIN
	return 0;
}

static uint8_t sens_itf_mote_pack_send_frame(sens_itf_cmd_req_t *cmd, uint8_t cmd_size)
{
	uint8_t size;

    size = sens_itf_pack_cmd_req(cmd, frame);

    if (size != cmd_size)
        return SENS_ITF_STATE_EXEC_ERROR;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_pt_val_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t size;
    uint8_t ans_size;

    point = acquisition_schedule.scan.index[st->point_index];
    ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (SENS_ITF_REGMAP_READ_POINT_DATA_1 + point))
        return SENS_ITF_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(sens_itf_cmd_point_t));

    st->retries = 0;
    st->point_index++;

    return SENS_ITF_STATE_EXEC_OK;
}


static uint8_t sens_itf_mote_sm_func_req_pt_val(sens_itf_mote_sm_state_t *st)
{
	uint8_t point;

	// end of point reading
    if(st->point_index >= acquisition_schedule.scan.num_of_points)
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
	st->retries++;
	if(st->retries > 3)
		return SENS_ITF_STATE_EXEC_ERROR;

	// normal point reading
	point = acquisition_schedule.scan.index[st->point_index];
    cmd.hdr.addr = SENS_ITF_REGMAP_READ_POINT_DATA_1 + point;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}

static uint8_t sens_itf_mote_sm_func_run_sch(sens_itf_mote_sm_state_t *st)
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
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;
    }
    else
    	return SENS_ITF_STATE_EXEC_WAIT_OK;
}

static uint8_t sens_itf_mote_sm_func_build_sch(sens_itf_mote_sm_state_t *st)
{
	uint8_t n, m;

    acquisition_schedule.num_of_points = 0;

    for (n = 0, m = 0; n < board_info.num_of_points; n++)
    {
        if ((sensor_points.points[n].desc.access_rights & SENS_ITF_ACCESS_READ_ONLY) &&
            (sensor_points.points[n].desc.sampling_time_x250ms > 0))
        {
            acquisition_schedule.points[m].index = n; 
            acquisition_schedule.points[m].counter = sensor_points.points[n].desc.sampling_time_x250ms;;
            acquisition_schedule.points[m].sampling_time_x250ms = sensor_points.points[n].desc.sampling_time_x250ms;

            m++;
            acquisition_schedule.num_of_points++;
        }
    }

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_pt_desc_ans(sens_itf_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 20;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size || (ans.hdr.addr != SENS_ITF_REGMAP_POINT_DESC_1 + st->point_index))
        return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&sensor_points.points[st->point_index].desc,&ans.payload.point_desc_cmd,sizeof(sens_itf_cmd_point_desc_t));

    st->point_index++;
    sensor_points.num_of_points = st->point_index;

    return SENS_ITF_STATE_EXEC_OK;
    }
    
static uint8_t sens_itf_mote_sm_func_req_pt_desc(sens_itf_mote_sm_state_t *st)
{
	// end of cyclic point description reading
    if(st->point_index >= board_info.num_of_points)
    	return SENS_ITF_STATE_EXEC_WAIT_ABORT;

    cmd.hdr.addr = SENS_ITF_REGMAP_POINT_DESC_1 + st->point_index;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}

static uint8_t sens_itf_mote_sm_func_proc_brd_id_ans(sens_itf_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 28;
    
    st->point_index = 0;
    
    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    
    if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(sens_itf_cmd_brd_id_t));

    if ((board_info.num_of_points == 0) || (board_info.num_of_points > SENS_ITF_MAX_POINTS))
    	return SENS_ITF_STATE_EXEC_ERROR;
    
    sensor_points.num_of_points = 0;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_req_brd_id(sens_itf_mote_sm_state_t *st)
{
	cmd.hdr.size = 4;
	cmd.hdr.addr = SENS_ITF_REGMAP_BRD_ID;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}

static uint8_t sens_itf_mote_sm_func_proc_itf_ver_ans(sens_itf_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 6;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    
    if (size != ans_size)
    	return SENS_ITF_STATE_EXEC_ERROR;

    if ((SENS_ITF_ANS_OK != ans.hdr.status) || (SENS_ITF_LATEST_VERSION != ans.payload.itf_version_cmd.version))
    	return SENS_ITF_STATE_EXEC_ERROR;

    return SENS_ITF_STATE_EXEC_OK;
}

static uint8_t sens_itf_mote_sm_func_wait_ans(sens_itf_mote_sm_state_t *st)
{
	if(st->frame_arrived)
		return SENS_ITF_STATE_EXEC_WAIT_STOP;

	st->trmout_counter++;

	if(st->trmout_counter > st->trmout)
		return SENS_ITF_STATE_EXEC_WAIT_ABORT;

	return SENS_ITF_STATE_EXEC_WAIT_OK;
}



static uint8_t sens_itf_mote_sm_func_req_ver(sens_itf_mote_sm_state_t *st)
{
    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return sens_itf_mote_pack_send_frame(&cmd, 4);
}


static uint8_t sens_itf_mote_sm_func_init(sens_itf_mote_sm_state_t *st)
{
	uint8_t ret = SENS_ITF_STATE_EXEC_OK;
    
	memset(&cmd, 0, sizeof(cmd));
	memset(&ans, 0, sizeof(ans));
    memset(&sensor_points, 0, sizeof(sensor_points));
	memset(&board_info, 0, sizeof(board_info));
	memset(&acquisition_schedule, 0, sizeof(acquisition_schedule));
	memset(st, 0, sizeof(sens_itf_mote_sm_state_t));

	return ret;
}

void sens_itf_mote_sm(void)
{
	uint8_t ret;
#ifdef TRACE_ON
	uint8_t ls = sm_state.state;
#endif

	ret = sens_itf_mote_sm_table[sm_state.state].func(&sm_state);
            
	switch(ret)
                {
		case SENS_ITF_STATE_EXEC_OK:
		case SENS_ITF_STATE_EXEC_WAIT_STOP:
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].next_state;
            break;
		case SENS_ITF_STATE_EXEC_WAIT_OK:
			// still waiting
            break;
		case SENS_ITF_STATE_EXEC_WAIT_ABORT:
			// wait timeout
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].abort_state;
            break;
		case SENS_ITF_STATE_EXEC_ERROR:
        default:
			sm_state.state = sens_itf_mote_sm_table[sm_state.state].error_state;
            break;
    }

	#ifdef TRACE_ON
    printf("sens_itf_mote_sm %d -> %d\n",ls,sm_state.state);
	#endif
}

sens_itf_mote_sm_table_t sens_itf_mote_sm_table[] =
{     //{ func,                                   next_state,                      abort_state,                error_state         }
		{ sens_itf_mote_sm_func_init,             SENS_ITF_STATE_SEND_ITF_VER,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_INIT
		{ sens_itf_mote_sm_func_req_ver,          SENS_ITF_STATE_WAIT_ITF_VER_ANS, SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_ITF_VER
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_ITF_VER,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_ITF_VER_ANS
		{ sens_itf_mote_sm_func_proc_itf_ver_ans, SENS_ITF_STATE_SEND_BRD_ID,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_ITF_VER
		{ sens_itf_mote_sm_func_req_brd_id,       SENS_ITF_STATE_WAIT_BRD_ID_ANS,  SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_BRD_ID
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_BRD_ID,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_BRD_ID_ANS
		{ sens_itf_mote_sm_func_proc_brd_id_ans,  SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_BRD_ID
		{ sens_itf_mote_sm_func_req_pt_desc,      SENS_ITF_STATE_WAIT_PT_DESC_ANS, SENS_ITF_STATE_BUILD_SCH,   SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_PT_DESC
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_PT_DESC_ANS
		{ sens_itf_mote_sm_func_pt_desc_ans,      SENS_ITF_STATE_SEND_PT_DESC,     SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_PT_DESC
		{ sens_itf_mote_sm_func_build_sch,        SENS_ITF_STATE_RUN_SCH,          SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_BUILD_SCH
		{ sens_itf_mote_sm_func_run_sch,          SENS_ITF_STATE_RUN_SCH,          SENS_ITF_STATE_SEND_PT_VAL, SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_RUN_SCH
		{ sens_itf_mote_sm_func_req_pt_val,       SENS_ITF_STATE_WAIT_PT_VAL_ANS,  SENS_ITF_STATE_RUN_SCH,     SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_SEND_PT_VAL
		{ sens_itf_mote_sm_func_wait_ans,         SENS_ITF_STATE_PROC_PT_VAL,      SENS_ITF_STATE_SEND_PT_VAL, SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_WAIT_PT_VAL_ANS
		{ sens_itf_mote_sm_func_pt_val_ans,       SENS_ITF_STATE_SEND_PT_VAL,      SENS_ITF_STATE_INIT,        SENS_ITF_STATE_INIT }, // SENS_ITF_STATE_PROC_PT_VAL
};
    
static void sens_itf_mote_tick(void)
    {   
    scheduler_push_task((task_cbt) sens_itf_mote_sm, TASKPRIO_SENS_ITF_MAIN);
    }

uint8_t sens_itf_mote_init(void)
{
	
	memset(&sm_state, 0, sizeof(sens_itf_mote_sm_state_t));
	sm_state.state = SENS_ITF_STATE_INIT;
    
    //opentimers_start(SENS_ITF_SM_TICK_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) sens_itf_mote_tick);

    return 0;
}


