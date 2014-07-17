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

owerror_t sens_itf_receive(
   OpenQueueEntry_t* msg,
   coap_header_iht*  coap_header,
   coap_option_iht*  coap_options
);
void sens_itf_sendDone(
   OpenQueueEntry_t* msg,
   owerror_t error
);

const uint8_t sens_itf_path0[]       = "s";
coap_resource_desc_t sens_itf_vars;
static sens_itf_cmd_brd_id_t board_info;
static sens_itf_point_ctrl_t sensor_points;
#define SENS_ITF_SENSOR_NUM_OF_POINTS 5

void sens_itf_init_point_db(void)
{
    uint8_t n;
    uint8_t *point_names[SENS_ITF_POINT_NAME_SIZE] = { "TEMP", "HUMID", "FIRE", "ALARM", "OPENCNT" };
    uint8_t data_types[SENS_ITF_POINT_NAME_SIZE] = {SENS_ITF_DT_FLOAT, SENS_ITF_DT_FLOAT, SENS_ITF_DT_U8,
        SENS_ITF_DT_U8, SENS_ITF_DT_U32};
    uint8_t access_rights[SENS_ITF_POINT_NAME_SIZE] = { SENS_ITF_ACCESS_READ_ONLY, SENS_ITF_ACCESS_READ_ONLY,
        SENS_ITF_ACCESS_READ_ONLY, SENS_ITF_ACCESS_WRITE_ONLY, SENS_ITF_ACCESS_READ_WRITE};
    uint32_t sampling_time[SENS_ITF_POINT_NAME_SIZE] = {4*10, 4*30, 4*1, 0, 0};

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
   
    sens_itf_mote_init();

    // prepare the resource descriptor for the /l path
    sens_itf_vars.path0len            = sizeof(sens_itf_path0)-1;
    sens_itf_vars.path0val            = (uint8_t*)(&sens_itf_path0);
    sens_itf_vars.path1len            = 0;
    sens_itf_vars.path1val            = NULL;
    sens_itf_vars.componentID         = COMPONENT_SENS_ITF;
    sens_itf_vars.callbackRx          = &sens_itf_receive;
    sens_itf_vars.callbackSendDone    = &sens_itf_sendDone;

    // register with the CoAP module
    opencoap_register(&sens_itf_vars);
}

//=========================== private =========================================

/**
\brief Called when a CoAP message is received for this resource.

\param[in] msg          The received message. CoAP header and options already
   parsed.
\param[in] coap_header  The CoAP header contained in the message.
\param[in] coap_options The CoAP options contained in the message.

\return Whether the response is prepared successfully.
*/
owerror_t sens_itf_receive(
      OpenQueueEntry_t* msg,
      coap_header_iht*  coap_header,
      coap_option_iht*  coap_options
   ) {
   owerror_t outcome;
   uint8_t n, nb;
   uint8_t buf[128];
   uint8_t *pbuf = &buf[0];
   
   switch (coap_header->Code) {
      case COAP_CODE_REQ_GET:
         // reset packet payload
         msg->payload                     = &(msg->packet[127]);
         msg->length                      = 0;
         
    	if(coap_options[1].length == 0) {
    		memcpy(pbuf,"{\"e\":[",6);
    		pbuf += 6;
    		//  { "n": "sensor1", "v":  val1, "u": "unity1" },
    		for (n = 0; n < SENS_ITF_SENSOR_NUM_OF_POINTS; n++){
    			memcpy(pbuf,"{\"n\":\"",6);
    			pbuf += 6;
    			nb = strlen(sensor_points.points[n].desc.name);
    			memcpy(pbuf,sensor_points.points[n].desc.name,nb);
    			pbuf += nb;
    			memcpy(pbuf,"\"}",2);
    			pbuf += 2;
    			if((SENS_ITF_SENSOR_NUM_OF_POINTS > 1) && (n < (SENS_ITF_SENSOR_NUM_OF_POINTS - 1))) {
    				memcpy(pbuf,",",1);
    				pbuf += 1;
    			}
    		}
    		memcpy(pbuf,"]}",2);
    		pbuf += 2;
    		nb = (uint8_t) (pbuf - buf);
    		// no options, return the full list
    		packetfunctions_reserveHeaderSize(msg,1+nb);
    		msg->payload[0] = COAP_PAYLOAD_MARKER;
    		memcpy(&msg->payload[1],pbuf,nb);
    	 }
    	else {

         // add CoAP payload
         packetfunctions_reserveHeaderSize(msg,1+11);
         msg->payload[0]                  = COAP_PAYLOAD_MARKER;
         memcpy(&msg->payload[1],"hello world",11);
            
         // set the CoAP header
         coap_header->Code                = COAP_CODE_RESP_CONTENT;
         
         outcome                          = E_SUCCESS;
    	}
         break;
      
      case COAP_CODE_REQ_PUT:
         // reset packet payload
         msg->payload                     = &(msg->packet[127]);
         msg->length                      = 0;
         
         // set the CoAP header
         coap_header->Code                = COAP_CODE_RESP_CHANGED;
         
         outcome                          = E_SUCCESS;
         break;
         
      default:
         outcome                          = E_FAIL;
         break;
   }
   
   return outcome;
}

/**
\brief The stack indicates that the packet was sent.

\param[in] msg The CoAP message just sent.
\param[in] error The outcome of sending it.
*/
void sens_itf_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}