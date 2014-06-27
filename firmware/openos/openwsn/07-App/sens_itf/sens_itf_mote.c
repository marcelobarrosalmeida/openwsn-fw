#include "sens_itf.h"

#define SENS_ITF_DBG_FRAME 0
#define SENS_ITF_OUTPUT    1

enum {
    SENS_ITF_STATE_NO_BRD = 0,
    SENS_ITF_STATE_ITF_VER,
    SENS_ITF_STATE_BRD_ID,
    SENS_ITF_STATE_DATA_POLLING,
    SENS_ITF_STATE_ERROR,
    SENS_ITF_STATE_IDLE
};

typedef struct sens_itf_acq_schedule_s
{
	uint8_t num_of_points;
	struct 
	{
		uint8_t index;
		uint32_t sampling_time_x250ms;
        uint32_t counter;
	} points[SENS_ITF_MAX_POINTS];
} sens_itf_acq_schedule_t;

static sens_itf_point_ctrl_t sensor_points;
static sens_itf_cmd_brd_id_t board_info;
static const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check sens_itf_datatypes_e order
static sens_itf_acq_schedule_t acquisition_schedule;
//static os_timer_t acquistion_timer = 0;
//static os_serial_t serial = 0;
static uint8_t frame[SENS_ITF_MAX_FRAME_SIZE];
static sens_itf_cmd_req_t cmd;
static sens_itf_cmd_res_t ans;

static uint8_t sens_itf_state;

static void sens_itf_mote_wake_up_sensor(void)
{

}

static void sens_itf_mote_sleep_sensor(void)
{

}

static uint8_t sens_itf_mote_send_frame(uint8_t *frame, uint8_t size)
{
    int16_t sent;
    return (sent < 0 ? 0 : (uint8_t) sent); // CHECK AGAIN
}

static uint8_t sens_itf_mote_recv_frame(uint8_t *frame, uint8_t size)
{
    int16_t recv;
    return (recv < 0 ? 0 : (uint8_t) recv); // CHECK AGAIN
}

static uint8_t sens_itf_mote_get_point_type(uint8_t point)
{
    return sensor_points.points[point].desc.type;
}



static uint8_t sens_itf_mote_get_points_desc(uint8_t point)
{
	uint8_t size;
    uint8_t cmd_size = 4;
    uint8_t ans_size = 20;

    cmd.hdr.addr = SENS_ITF_REGMAP_POINT_DESC_1 + point;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    os_kernel_sleep(2000);

    size = sens_itf_mote_recv_frame(frame, ans_size);
    if (size != ans_size)
        return 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    return 1;
}

static uint8_t sens_itf_mote_get_points_value(uint8_t point)
{
	uint8_t size;
    uint8_t cmd_size = 4;
    uint8_t ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    cmd.hdr.addr = SENS_ITF_REGMAP_READ_POINT_DATA_1 + point;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    os_kernel_sleep(2000);

    size = sens_itf_mote_recv_frame(frame, ans_size);
    if (size != ans_size)
        return 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    return 1;
}

static uint8_t sens_itf_mote_set_points_value(uint8_t point)
{
	uint8_t size;
    uint8_t cmd_size = 5 + datatype_sizes[sensor_points.points[point].desc.type];
    uint8_t ans_size = 5;

    cmd.hdr.addr = SENS_ITF_REGMAP_WRITE_POINT_DATA_1 + point;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    os_kernel_sleep(2000);

    size = sens_itf_mote_recv_frame(frame, ans_size);
    if (size != ans_size)
        return 0;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    return 1;
}

static void sens_itf_mote_build_acquisition_schedule(void)
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
}

static void sens_itf_mote_read_point(uint8_t index)
{
    /*
    NEXT STEP: JUST ADD A NEW TASK TO SCHEDULER, DO NOT READ THE POINT
    */
    uint8_t ret;
    OS_UTIL_LOG(1,("Reading point %d\n", index));
    ret = sens_itf_mote_get_points_value(index);
    if (ret)
    {
        // update
        memcpy(&sensor_points.points[index].value, &ans.payload.point_value_cmd, sizeof(sens_itf_cmd_point_t));
    }

}

static void sens_itf_mote_acquisition_timer_func(void)
{
    uint8_t n;

    for (n = 0; n < acquisition_schedule.num_of_points; n++)
    {
        acquisition_schedule.points[n].counter--;
        if (acquisition_schedule.points[n].counter == 0)
        {
            sens_itf_mote_read_point(acquisition_schedule.points[n].index);
            acquisition_schedule.points[n].counter = acquisition_schedule.points[n].sampling_time_x250ms;
        }
    }
    
}

static uint8_t sens_itf_mote_request_version(void)
{
    uint8_t size;
    uint8_t cmd_size = 4;
    
    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;
    
    size = sens_itf_pack_cmd_req(&cmd, frame);
    
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;
    
    return 1;
}

static uint8_t sens_itf_mote_check_version(void)
{
    uint8_t size;
    uint8_t ans_size = 6;

    cmd.hdr.addr = SENS_ITF_REGMAP_ITF_VERSION;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    
    if (size != ans_size)
        return 0;

    if ((SENS_ITF_ANS_OK == ans.hdr.status) && (SENS_ITF_LATEST_VERSION == ans.payload.itf_version_cmd.version))
        return 1;
    else
        return 0;

}

static uint8_t sens_itf_mote_request_board_id(void)
{
	uint8_t size;
    uint8_t cmd_size = 4;

	cmd.hdr.size = cmd_size;
	cmd.hdr.addr = SENS_ITF_REGMAP_BRD_ID;

    size = sens_itf_pack_cmd_req(&cmd, frame);
    if (size != cmd_size)
        return 0;

    if (sens_itf_mote_send_frame(frame, cmd_size) != cmd_size)
        return 0;

    return 1;
}

static uint8_t sens_itf_mote_check_board_id(void)
{
	uint8_t size;
    uint8_t ans_size = 28;

    size = sens_itf_unpack_cmd_res(&ans, frame, ans_size);
    if (size != ans_size)
        return 0;

    memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(sens_itf_cmd_brd_id_t));
    
    if ((board_info.num_of_points > 0) && (board_info.num_of_points <= SENS_ITF_MAX_POINTS))
        return 1;
    else
        return 0;

}

void sens_itf_mote_main(void)
{
#ifdef TRACE_ON
    printf("sens_itf_mote_main\n");
#endif

    switch(sens_itf_state)
    {
        case SENS_ITF_STATE_NO_BRD:
            if(sens_itf_mote_request_version())
            {
                sens_itf_timeout = 0;
                sens_itf_frame_received = 0;
                sens_itf_state = SENS_ITF_STATE_ITF_VER;
            }
            break;
        case SENS_ITF_STATE_ITF_VER:
            if(sens_itf_frame_received)
            {
                if(sens_itf_mote_check_version())
                {
                    sens_itf_timeout = 0;
                    sens_itf_frame_received = 0;
                    sens_itf_state = SENS_ITF_STATE_BRD_ID;
                    sens_itf_mote_request_board_id();
                }
                else
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            else
            {
                sens_itf_timeout++;
                if(sens_itf_timeout >= 3)
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            
            break;
        case SENS_ITF_STATE_BRD_ID:
            if(sens_itf_frame_received)
            {
                if(sens_itf_mote_check_board_id())
                {
                    sens_itf_timeout = 0;
                    sens_itf_frame_received = 0;
                    sens_itf_point_desc = 0;
                    sens_itf_state = SENS_ITF_STATE_GET_POINT_DESC;
                    sens_itf_mote_request_point_desc(sens_itf_point_desc);
                }
                else
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            else
            {
                sens_itf_timeout++;
                if(sens_itf_timeout >= 3)
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            
            break;
        case SENS_ITF_STATE_GET_POINT_DESC:
            if(sens_itf_frame_received)
            {
                if(sens_itf_mote_check_point_desc())
                {
                    sens_itf_timeout = 0;
                    sens_itf_frame_received = 0;
                    sens_itf_point_desc++;
                    if(sens_itf_point_desc >= board_info.num_of_points)
                    {
                        sens_itf_mote_build_acquisition_schedule();
                        if (acquisition_schedule.num_of_points > 0)
                        {
                            sens_itf_state = SENS_ITF_STATE_DATA_POLLING;
                        }
                        else
                        {
                            sens_itf_state = SENS_ITF_STATE_IDLE;
                        }
                    }
                    else
                    {
                        sens_itf_mote_request_point_desc(sens_itf_point_desc);
                    }
                }
            }        
            else
            {
                sens_itf_timeout++;
                if(sens_itf_timeout >= 3)
                {
                    sens_itf_state = SENS_ITF_STATE_NO_BRD;
                }
            }
            
            break;
        case SENS_ITF_STATE_DATA_POLLING:
            break;
        case SENS_ITF_STATE_IDLE:
        case SENS_ITF_STATE_ERROR:
        default:
            break;
    }
}



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



void sens_itf_mote_init(void)
{
    uint8_t n;
    uint8_t ret = 0;

	memset(&cmd, 0, sizeof(cmd));
	memset(&ans, 0, sizeof(ans));

    memset(&sensor_points, 0, sizeof(sensor_points));
	memset(&board_info, 0, sizeof(board_info));
	memset(&acquisition_schedule, 0, sizeof(acquisition_schedule));

    sens_itf_state = SENS_ITF_STATE_NO_BRD;    
}
