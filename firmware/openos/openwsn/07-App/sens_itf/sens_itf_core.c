#include <string.h>
#include <stdint.h>
#include "openwsn.h"
#include "opencoap.h"
#include "openqueue.h"
#include "sens_itf.h"
#include "packetfunctions.h"
#include "opentimers.h"
#include "scheduler.h"

#define TRACE_ON 1

owerror_t sens_itf_desc_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void sens_itf_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error);
owerror_t sens_itf_val_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void sens_itf_val_sendDone(OpenQueueEntry_t* msg, owerror_t error);

const uint8_t sens_itf_desc_path0 [] = "d";
coap_resource_desc_t sens_itf_desc_vars;
const uint8_t sens_itf_val_path0 [] = "s";
coap_resource_desc_t sens_itf_val_vars;
static sens_itf_cmd_brd_id_t board_info;
static sens_itf_point_ctrl_t sensor_points;
#define SENS_ITF_SENSOR_NUM_OF_POINTS 5

#include "adaptive_sync.h"
extern adaptive_sync_vars_t adaptive_sync_vars;

void sens_itf_init_point_db(void)
{
    uint8_t n;
    uint8_t *point_names[SENS_ITF_POINT_NAME_SIZE] = { "TEMP", "HUMID", "FIRE", "ALARM", "OPENCNT" };
    uint8_t data_types[SENS_ITF_POINT_NAME_SIZE] = { SENS_ITF_DT_FLOAT, SENS_ITF_DT_FLOAT, SENS_ITF_DT_U8,
        SENS_ITF_DT_U8, SENS_ITF_DT_U32 };
    uint8_t access_rights[SENS_ITF_POINT_NAME_SIZE] = { SENS_ITF_ACCESS_READ_ONLY, SENS_ITF_ACCESS_READ_ONLY,
        SENS_ITF_ACCESS_READ_ONLY, SENS_ITF_ACCESS_WRITE_ONLY, SENS_ITF_ACCESS_READ_WRITE };
    uint32_t sampling_time[SENS_ITF_POINT_NAME_SIZE] = { 4 * 10, 4 * 30, 4 * 1, 0, 0 };

    memset(&sensor_points, 0, sizeof(sensor_points));
    memset(&board_info, 0, sizeof(board_info));

    strcpy(board_info.model, "OWSN");
    strcpy(board_info.manufactor, "UFUSP");
    board_info.sensor_id = 0xDEADBEEF;
    board_info.hardware_revision = 0x01;
    board_info.num_of_points = SENS_ITF_SENSOR_NUM_OF_POINTS;
    board_info.cabalities = SENS_ITF_CAPABILITIES_DISPLAY |
        SENS_ITF_CAPABILITIES_WPAN_STATUS |
        SENS_ITF_CAPABILITIES_BATTERY_STATUS;

    sensor_points.num_of_points = SENS_ITF_SENSOR_NUM_OF_POINTS;

    for (n = 0; n < SENS_ITF_SENSOR_NUM_OF_POINTS; n++)
    {
        strcpy(sensor_points.points[n].desc.name, point_names[n]);
        sensor_points.points[n].desc.type = data_types[n];
        sensor_points.points[n].desc.unit = 0; // TDB
        sensor_points.points[n].desc.access_rights = access_rights[n];
        sensor_points.points[n].desc.sampling_time_x250ms = sampling_time[n];
        sensor_points.points[n].value.type = data_types[n];
    }
}

void sens_itf_core_init(void) {

    sens_itf_init_point_db();
    //sens_itf_mote_init();

    // prepare the resource descriptor for the /d and /s paths
    sens_itf_desc_vars.path0len = sizeof(sens_itf_desc_path0) -1;
    sens_itf_desc_vars.path0val = (uint8_t*) (&sens_itf_desc_path0);
    sens_itf_desc_vars.path1len = 0;
    sens_itf_desc_vars.path1val = NULL;
    sens_itf_desc_vars.componentID = COMPONENT_SENS_ITF;
    sens_itf_desc_vars.callbackRx = &sens_itf_desc_receive;
    sens_itf_desc_vars.callbackSendDone = &sens_itf_desc_sendDone;

    sens_itf_val_vars.path0len = sizeof(sens_itf_val_path0) -1;
    sens_itf_val_vars.path0val = (uint8_t*) (&sens_itf_val_path0);
    sens_itf_val_vars.path1len = 0;
    sens_itf_val_vars.path1val = NULL;
    sens_itf_val_vars.componentID = COMPONENT_SENS_ITF;
    sens_itf_val_vars.callbackRx = &sens_itf_val_receive;
    sens_itf_val_vars.callbackSendDone = &sens_itf_val_sendDone;

    // register with the CoAP modules
    opencoap_register(&sens_itf_desc_vars);
    opencoap_register(&sens_itf_val_vars);
}

//=========================== private =========================================

owerror_t sens_itf_desc_receive(
    OpenQueueEntry_t* msg,
    coap_header_iht*  coap_header,
    coap_option_iht*  coap_options
    ) {
    owerror_t outcome = E_FAIL;
    uint8_t n, nb;
    uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code) {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        if (coap_options[1].length == 0) {
            n = snprintf(buf, 80, "{\"ver\":%u,\"model\":\"%s\",\"id\":%u,\"npts\":%u}",
                SENS_ITF_LATEST_VERSION,
                board_info.model,
                board_info.sensor_id,
                SENS_ITF_SENSOR_NUM_OF_POINTS);

            packetfunctions_reserveHeaderSize(msg, 1 + n);
            msg->payload[0] = COAP_PAYLOAD_MARKER;

            memcpy(&msg->payload[1], buf, n);
            coap_header->Code = COAP_CODE_RESP_CONTENT;
            outcome = E_SUCCESS;
        }

        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // set the CoAP header
        coap_header->Code = COAP_CODE_RESP_CHANGED;

        outcome = E_SUCCESS;
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

owerror_t sens_itf_val_receive(
    OpenQueueEntry_t* msg,
    coap_header_iht*  coap_header,
    coap_option_iht*  coap_options
    ) {
    owerror_t outcome = E_FAIL;
    uint8_t n, nb;
    uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code) {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        if (coap_options[1].length == 0) {
            n = snprintf(buf, 80, "[{\"temp\":%u},{\"hum\":\"%u%%\"},{\"alarm\":%u}]", 
                ieee154e_vars.asn.bytes0and1 & 0xff,
                (ieee154e_vars.asn.bytes0and1 >> 8) & 0xff,
                ieee154e_vars.asn.bytes2and3 & 0xff);

            packetfunctions_reserveHeaderSize(msg, 1 + n);
            msg->payload[0] = COAP_PAYLOAD_MARKER;

            memcpy(&msg->payload[1], buf, n );
            coap_header->Code = COAP_CODE_RESP_CONTENT;
            outcome = E_SUCCESS;
        }

        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // set the CoAP header
        coap_header->Code = COAP_CODE_RESP_CHANGED;

        outcome = E_SUCCESS;
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

void sens_itf_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

void sens_itf_val_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}
