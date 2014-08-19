#include <string.h>
#include <stdint.h>
#include "osens.h"
#include "osens_itf.h"
#include "buf_io.h"
#include "crc16.h"

#define OSENS_DBG_FRAME 1

uint8_t osens_unpack_point_value(osens_point_t *point, uint8_t *buf)
{
    uint8_t size = 0;

    switch (point->type)
    {
    case OSENS_DT_U8:
        point->value.u8 = buf_io_get8_fl(buf);
        size = 1;
        break;
    case OSENS_DT_S8:
        point->value.s8 = buf_io_get8_fl(buf);
        size = 1;
        break;
    case OSENS_DT_U16:
        point->value.u16 = buf_io_get16_fl(buf);
        size = 2;
        break;
    case OSENS_DT_S16:
        point->value.s16 = buf_io_get16_fl(buf);
        size = 2;
        break;
    case OSENS_DT_U32:
        point->value.u32 = buf_io_get32_fl(buf);
        size = 4;
        break;
    case OSENS_DT_S32:
        point->value.s32 = buf_io_get32_fl(buf);
        size = 4;
        break;
    case OSENS_DT_U64:
        point->value.u64 = buf_io_get64_fl(buf);
        size = 8;
        break;
    case OSENS_DT_S64:
        point->value.s64 = buf_io_get64_fl(buf);
        size = 8;
        break;
    case OSENS_DT_FLOAT:
        point->value.fp32 = buf_io_getf_fl(buf);
        size = 4;
        break;
    case OSENS_DT_DOUBLE:
        point->value.fp64 = buf_io_getd_fl(buf);
        size = 8;
        break;
    default:
        break;
    }

    return size;
}

uint8_t osens_pack_point_value(const osens_point_t *point, uint8_t *buf)
{
    uint8_t size = 0;

    switch (point->type)
    {
    case OSENS_DT_U8:
        buf_io_put8_tl(point->value.u8, buf);
        size = 1;
        break;
    case OSENS_DT_S8:
        buf_io_put8_tl(point->value.s8, buf);
        size = 1;
        break;
    case OSENS_DT_U16:
        buf_io_put16_tl(point->value.u16, buf);
        size = 2;
        break;
    case OSENS_DT_S16:
        buf_io_put16_tl(point->value.s16, buf);
        size = 2;
        break;
    case OSENS_DT_U32:
        buf_io_put32_tl(point->value.u32, buf);
        size = 4;
        break;
    case OSENS_DT_S32:
        buf_io_put32_tl(point->value.s32, buf);
        size = 4;
        break;
    case OSENS_DT_U64:
        buf_io_put64_tl(point->value.u64, buf);
        size = 8;
        break;
    case OSENS_DT_S64:
        buf_io_put64_tl(point->value.s64, buf);
        size = 8;
        break;
    case OSENS_DT_FLOAT:
        buf_io_putf_tl(point->value.fp32, buf);
        size = 4;
        break;
    case OSENS_DT_DOUBLE:
        buf_io_putd_tl(point->value.fp64, buf);
        size = 8;
        break;
    default:
        break;
    }

    return size;
}

uint8_t osens_unpack_cmd_req(osens_cmd_req_t *cmd, uint8_t *frame, uint8_t frame_size)
{
    uint8_t *buf = frame;
    uint16_t crc;
    uint16_t frame_crc;
    uint8_t size;

    if (frame_size < 3)
    {
        //OS_UTIL_LOG(OSENS_DBG_FRAME, ("Invalid frame size %d", frame_size));
        return 0;
    }
    
    // minimal header decoding
    cmd->hdr.size = buf_io_get8_fl_ap(buf);
    cmd->hdr.addr = buf_io_get8_fl_ap(buf);

    frame_crc = buf_io_get16_fl(&frame[cmd->hdr.size]);
    crc = crc16_calc(frame, cmd->hdr.size);
    cmd->crc = frame_crc;

    if (frame_crc != crc)
    {
        //OS_UTIL_LOG(OSENS_DBG_FRAME, ("Invalid CRC %04X <> %04X", frame_crc, crc));
        return 0;
    }
    
    switch (cmd->hdr.addr)
    {
    case OSENS_REGMAP_BRD_CMD:
        cmd->payload.command_cmd.cmd = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_WRITE_BAT_STATUS:
        cmd->payload.bat_status_cmd.status = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_WRITE_BAT_CHARGE:
        cmd->payload.bat_charge_cmd.charge = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_DSP_WRITE:
        cmd->payload.write_display_cmd.line = buf_io_get8_fl_ap(buf);
        memcpy(cmd->payload.write_display_cmd.msg,buf,OSENS_DSP_MSG_MAX_SIZE);
        buf += OSENS_DSP_MSG_MAX_SIZE;
        break;
    case OSENS_REGMAP_WPAN_STATUS:
        cmd->payload.wpan_status_cmd.status = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_WPAN_STRENGTH:
        cmd->payload.wpan_strength_cmd.strenght = buf_io_get8_fl_ap(buf);
        break;
    default:
        break;
    }

    if ((cmd->hdr.addr >= OSENS_REGMAP_WRITE_POINT_DATA_1) && 
        (cmd->hdr.addr <= OSENS_REGMAP_WRITE_POINT_DATA_32))
    {
        //uint8_t point = cmd->hdr.addr - OSENS_REGMAP_WRITE_POINT_DATA_1;
        cmd->payload.point_value_cmd.type =  buf_io_get8_fl_ap(buf);
        buf += osens_unpack_point_value(&cmd->payload.point_value_cmd, buf);
    }

    size = cmd->hdr.size + 2; // + crc 
    return size;
}

uint8_t osens_pack_cmd_res(osens_cmd_res_t *cmd, uint8_t *frame)
{
    uint8_t *buf = &frame[1];
    uint8_t size = 0;
    uint16_t crc;

    buf_io_put8_tl_ap(cmd->hdr.addr, buf);
    buf_io_put8_tl_ap(cmd->hdr.status, buf);

    // only fill command when status is OK, otherwise an error will be reported
    if (cmd->hdr.status == OSENS_ANS_OK)
    {
        switch (cmd->hdr.addr)
        {
        case OSENS_REGMAP_ITF_VERSION:
            buf_io_put8_tl_ap(cmd->payload.itf_version_cmd.version, buf);
            break;
        case OSENS_REGMAP_BRD_ID:
            memcpy(buf, cmd->payload.brd_id_cmd.model, OSENS_MODEL_NAME_SIZE);
            buf += OSENS_MODEL_NAME_SIZE;
            memcpy(buf, cmd->payload.brd_id_cmd.manufactor, OSENS_MANUF_NAME_SIZE);
            buf += OSENS_MANUF_NAME_SIZE;
            buf_io_put32_tl_ap(cmd->payload.brd_id_cmd.sensor_id, buf);
            buf_io_put8_tl_ap(cmd->payload.brd_id_cmd.hardware_revision, buf);
            buf_io_put8_tl_ap(cmd->payload.brd_id_cmd.num_of_points, buf);
            buf_io_put8_tl_ap(cmd->payload.brd_id_cmd.capabilities, buf);
            break;
        case OSENS_REGMAP_BRD_STATUS:
            buf_io_put8_tl_ap(cmd->payload.brd_status_cmd.status, buf);
            break;
        case OSENS_REGMAP_BRD_CMD:
            buf_io_put8_tl_ap(cmd->payload.command_res_cmd.status, buf);
            break;
        case OSENS_REGMAP_READ_BAT_STATUS:
            buf_io_put8_tl_ap(cmd->payload.bat_status_cmd.status, buf);
            break;
        case OSENS_REGMAP_READ_BAT_CHARGE:
            buf_io_put8_tl_ap(cmd->payload.bat_charge_cmd.charge, buf);
            break;
        case OSENS_REGMAP_SVR_MAIN_ADDR:
        case OSENS_REGMAP_SVR_SEC_ADDR:
            memcpy(buf, cmd->payload.svr_addr_cmd.addr, OSENS_SERVER_ADDR_SIZE);
            buf += OSENS_SERVER_ADDR_SIZE;
            break;
        default:
            break;
        }

        if ((cmd->hdr.addr >= OSENS_REGMAP_POINT_DESC_1) &&
            (cmd->hdr.addr <= OSENS_REGMAP_POINT_DESC_32))
        {
            //uint8_t point = cmd->hdr.addr - OSENS_REGMAP_POINT_DESC_1;
            memcpy(buf, cmd->payload.point_desc_cmd.name, OSENS_POINT_NAME_SIZE);
            buf += OSENS_POINT_NAME_SIZE;
            buf_io_put8_tl_ap(cmd->payload.point_desc_cmd.type, buf);
            buf_io_put8_tl_ap(cmd->payload.point_desc_cmd.unit, buf);
            buf_io_put8_tl_ap(cmd->payload.point_desc_cmd.access_rights, buf);
            buf_io_put32_tl_ap(cmd->payload.point_desc_cmd.sampling_time_x250ms, buf);
        }

        if ((cmd->hdr.addr >= OSENS_REGMAP_READ_POINT_DATA_1) &&
            (cmd->hdr.addr <= OSENS_REGMAP_READ_POINT_DATA_32))
        {
            //uint8_t point = cmd->hdr.addr - OSENS_REGMAP_READ_POINT_DATA_1;
            buf_io_put8_tl_ap(cmd->payload.point_value_cmd.type,buf);
            buf += osens_pack_point_value(&cmd->payload.point_value_cmd, buf);
        }
    }

    size = buf - frame;
    buf_io_put8_tl(size, frame);
    crc = crc16_calc(frame, size);
    cmd->crc = crc;
    cmd->hdr.size = size;
    buf_io_put16_tl(crc, buf);
    
    size += 2; // +crc 
    return size;
}


uint8_t osens_unpack_cmd_res(osens_cmd_res_t * cmd, uint8_t *frame, uint8_t frame_size)
{
    uint8_t size;
    uint8_t *buf = frame;
    uint16_t crc;
    uint16_t frame_crc;

    if (frame_size < 3)
    {
        cmd->hdr.status = OSENS_ANS_ERROR;
        return 0;
    }
    
    // minimal header decoding
    cmd->hdr.size = buf_io_get8_fl_ap(buf);
    cmd->hdr.addr = buf_io_get8_fl_ap(buf);
    cmd->hdr.status = buf_io_get8_fl_ap(buf);


    frame_crc = buf_io_get16_fl(&frame[cmd->hdr.size]);
    crc = crc16_calc(frame, cmd->hdr.size);
    cmd->crc = frame_crc;

    if (frame_crc != crc)
    {
        //OS_UTIL_LOG(OSENS_DBG_FRAME, ("Invalid CRC %04X <> %04X", frame_crc, crc));
        cmd->hdr.status = OSENS_ANS_CRC_ERROR;
        return 0;
    }

    if (cmd->hdr.status != OSENS_ANS_OK)
    {
        //OS_UTIL_LOG(OSENS_DBG_FRAME, ("Response error %d", cmd->hdr.status));
        return 0;
    }


    switch (cmd->hdr.addr)
    {
    case OSENS_REGMAP_ITF_VERSION:
        cmd->payload.itf_version_cmd.version = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_BRD_ID:
        memcpy(cmd->payload.brd_id_cmd.model, buf, OSENS_MODEL_NAME_SIZE);
        buf += OSENS_MODEL_NAME_SIZE;
        memcpy(cmd->payload.brd_id_cmd.manufactor, buf, OSENS_MANUF_NAME_SIZE);
        buf += OSENS_MANUF_NAME_SIZE;
        cmd->payload.brd_id_cmd.sensor_id = buf_io_get32_fl_ap(buf);
        cmd->payload.brd_id_cmd.hardware_revision = buf_io_get8_fl_ap(buf);
        cmd->payload.brd_id_cmd.num_of_points = buf_io_get8_fl_ap(buf);
        cmd->payload.brd_id_cmd.capabilities = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_BRD_STATUS:
        cmd->payload.brd_status_cmd.status = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_BRD_CMD:
        cmd->payload.command_res_cmd.status = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_READ_BAT_STATUS:
        cmd->payload.bat_status_cmd.status = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_READ_BAT_CHARGE:
        cmd->payload.bat_charge_cmd.charge = buf_io_get8_fl_ap(buf);
        break;
    case OSENS_REGMAP_SVR_MAIN_ADDR:
    case OSENS_REGMAP_SVR_SEC_ADDR:
        memcpy(cmd->payload.svr_addr_cmd.addr, buf, OSENS_SERVER_ADDR_SIZE);
        buf += OSENS_SERVER_ADDR_SIZE;
        break;
    default:
        break;
    }

    if ((cmd->hdr.addr >= OSENS_REGMAP_POINT_DESC_1) && 
        (cmd->hdr.addr <= OSENS_REGMAP_POINT_DESC_32))
    {
        memcpy(cmd->payload.point_desc_cmd.name, buf, OSENS_POINT_NAME_SIZE);
        buf += OSENS_POINT_NAME_SIZE;
        cmd->payload.point_desc_cmd.type = buf_io_get8_fl_ap(buf);
        cmd->payload.point_desc_cmd.unit = buf_io_get8_fl_ap(buf);
        cmd->payload.point_desc_cmd.access_rights = buf_io_get8_fl_ap(buf);
        cmd->payload.point_desc_cmd.sampling_time_x250ms = buf_io_get32_fl_ap(buf);
    }

    if ((cmd->hdr.addr >= OSENS_REGMAP_READ_POINT_DATA_1) &&
        (cmd->hdr.addr <= OSENS_REGMAP_READ_POINT_DATA_32))
    {
        cmd->payload.point_value_cmd.type = buf_io_get8_fl_ap(buf);
        buf += osens_unpack_point_value(&cmd->payload.point_value_cmd, buf);
    }

    size = cmd->hdr.size + 2; // crc 
    return size;
}

uint8_t osens_pack_cmd_req(osens_cmd_req_t *cmd, uint8_t *frame)
{
    uint8_t *buf = &frame[1];
    uint8_t size = 0;
    uint16_t crc;

    // address
    // commands without arguments are handled only with this line
    buf_io_put8_tl_ap(cmd->hdr.addr, buf);
    
    switch (cmd->hdr.addr)
    {
    case OSENS_REGMAP_BRD_CMD:
        buf_io_put8_tl_ap(cmd->payload.command_cmd.cmd, buf);
        break;
    case OSENS_REGMAP_WRITE_BAT_STATUS:
        buf_io_put8_tl_ap(cmd->payload.bat_status_cmd.status, buf);
        break;
    case OSENS_REGMAP_WRITE_BAT_CHARGE:
        buf_io_put8_tl_ap(cmd->payload.bat_charge_cmd.charge, buf);
        break;
    case OSENS_REGMAP_DSP_WRITE:
        buf_io_put8_tl_ap(cmd->payload.write_display_cmd.line, buf);
        memcpy(buf, cmd->payload.write_display_cmd.msg, OSENS_DSP_MSG_MAX_SIZE);
        buf += OSENS_DSP_MSG_MAX_SIZE;
        break;
    case OSENS_REGMAP_WPAN_STATUS:
        buf_io_put8_tl_ap(cmd->payload.wpan_status_cmd.status,buf);
        break;
    case OSENS_REGMAP_WPAN_STRENGTH:
        buf_io_put8_tl_ap(cmd->payload.wpan_strength_cmd.strenght,buf);
        break;
    default:
        break;
    }

    if ((cmd->hdr.addr >= OSENS_REGMAP_WRITE_POINT_DATA_1) && 
        (cmd->hdr.addr <= OSENS_REGMAP_WRITE_POINT_DATA_32))
    {
        buf_io_put8_tl_ap(cmd->payload.point_value_cmd.type, buf);
        buf += osens_pack_point_value(&cmd->payload.point_value_cmd, buf);
    }

    size = buf - frame;
    buf_io_put8_tl(size, frame);
    crc = crc16_calc(frame, size);
    cmd->crc = crc;
    cmd->hdr.size = size;
    buf_io_put16_tl(crc, buf);

    size += 2; // + crc

    return size;
}
