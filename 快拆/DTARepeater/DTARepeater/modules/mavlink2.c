#include "modules.h"
#include "mavlink2.h"

#include "common/mavlink.h"

#include "usart.h"

extern UART_HandleTypeDef huart3;
mavlink_message_t msg;
mavlink_gimbal_control_standard_t cmd;
float gain_yaw = 100/512.0f;
float gain_pitch = 100 / 512.0f;
float max_pitch = 15 * 90 / 512.0f;

#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) HAL_UART_Transmit(&huart1,buf,len,1000);
//#define MAVLINK_SEND_UART_BYTES(chan, buf, len) Sent_uart1((uint8_t * )buf, len);
#define MAVLINK_MAX_PAYLOAD_LEN 255
void gimbal_control_standard_send(mavlink_channel_t chan, uint8_t priority, uint8_t yaw_mode, uint8_t pitch_mode, uint8_t roll_mode, float yaw_channel, float pitch_channel, float roll_channel, float drones_yawvelocity_desire);
#define		MSG_LEN_MAX			(255+8+1)
uint8_t	txtemp[MSG_LEN_MAX];

float _yaw = 0;
float _pitch = 0;

void send_cmd(float yaw,float pitch)
{
    if(pitch - 512 < -40)
    {
        _pitch = (pitch - 512 + 40)  * gain_pitch * 0.3;
    }
    else if(pitch - 512 > 40)
    {
        _pitch = (pitch - 512 - 40)  * gain_pitch * 0.3;
    }
    else
    {
        _pitch =0;
    }

    if(yaw - 512 < -40)
    {
        _yaw = (yaw - 512 + 40) * gain_yaw * 0.3;
    }
    else if(yaw - 512 > 40)
    {
        _yaw = (yaw - 512 - 40) * gain_yaw * 0.3;
    }
    else
    {
        _yaw =0;
    }

//    _yaw = _yaw * 0.9 + (yaw - 512) * gain_yaw * 0.1;

//    if(pitch - 512 > max_pitch)
//    {
//        _pitch = max_pitch;
//    }
//    else
//    {
//        _pitch = _pitch * 0.9 + (pitch - 512) * gain_pitch * 0.1;
//    }

    gimbal_control_standard_send((mavlink_channel_t)0, (uint8_t)20, (uint8_t)3, (uint8_t)3, (uint8_t)4, (float)_yaw, (float)_pitch, (float)0, (float)0);
}

//void gimbal_control_standard_send(mavlink_channel_t chan, uint8_t priority, uint8_t yaw_mode, uint8_t pitch_mode, uint8_t roll_mode, float yaw_channel, float pitch_channel, float roll_channel, float drones_yawvelocity_desire)

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
MAVLINK_HELPER void _chan_send(mavlink_channel_t chan, 
                                uint32_t msgid, 
                                const char *packet, 
                                uint8_t min_length, 
                                uint8_t length, 
                                uint8_t crc_extra)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(chan);
        uint8_t header_len = MAVLINK_CORE_HEADER_LEN;
	uint8_t signature_len = 0;
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
	bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
	bool signing = 	(!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);

    if (mavlink1)
    {
        length = min_length;
        if (msgid > 255)
        {
            // can't send 16 bit messages
            _mav_parse_error(status);
            return;
        }
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
        buf[0] = MAVLINK_STX_MAVLINK1;
        buf[1] = length;
        buf[2] = status->current_tx_seq;
        buf[3] = 1;
        buf[4] = 154;
        buf[5] = msgid & 0xFF;
    }
    else
    {
	    uint8_t incompat_flags = 0;
	    if (signing)
        {
            incompat_flags |= MAVLINK_IFLAG_SIGNED;
	    }
        length = _mav_trim_payload(packet, length);
        buf[0] = MAVLINK_STX;
        buf[1] = length;
        buf[2] = incompat_flags;
        buf[3] = 0; // compat_flags
        buf[4] = status->current_tx_seq;
        buf[5] = 1;
        buf[6] = 154;
        buf[7] = msgid & 0xFF;
        buf[8] = (msgid >> 8) & 0xFF;
        buf[9] = (msgid >> 16) & 0xFF;
    }
	status->current_tx_seq++;
	checksum = crc_calculate((const uint8_t*)&buf[1], header_len);
	crc_accumulate_buffer(&checksum, packet, length);
	crc_accumulate(crc_extra, &checksum);
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	if (signing)
    {
		// possibly add a signature
		signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len+1,
						    (const uint8_t *)packet, length, ck);
	}

//	MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
	MAVLINK_SEND_UART_BYTES(chan, buf, header_len+1);
	MAVLINK_SEND_UART_BYTES(chan, (uint8_t*)packet, length);
	MAVLINK_SEND_UART_BYTES(chan, ck, 2);
	if (signature_len != 0)
    {
		MAVLINK_SEND_UART_BYTES(chan, signature, signature_len);
	}
//	MAVLINK_END_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
}


void gimbal_control_standard_send(mavlink_channel_t chan, uint8_t priority, uint8_t yaw_mode, uint8_t pitch_mode, uint8_t roll_mode, float yaw_channel, float pitch_channel, float roll_channel, float drones_yawvelocity_desire)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_LEN];
    _mav_put_float(buf, 0, yaw_channel);
    _mav_put_float(buf, 4, pitch_channel);
    _mav_put_float(buf, 8, roll_channel);
    _mav_put_float(buf, 12, drones_yawvelocity_desire);
    _mav_put_uint8_t(buf, 16, priority);
    _mav_put_uint8_t(buf, 17, yaw_mode);
    _mav_put_uint8_t(buf, 18, pitch_mode);
    _mav_put_uint8_t(buf, 19, roll_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD, buf, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_CRC);
#else
    mavlink_gimbal_control_standard_t packet;
    packet.yaw_channel = yaw_channel;
    packet.pitch_channel = pitch_channel;
    packet.roll_channel = roll_channel;
    packet.drones_yawvelocity_desire = drones_yawvelocity_desire;
    packet.priority = priority;
    packet.yaw_mode = yaw_mode;
    packet.pitch_mode = pitch_mode;
    packet.roll_mode = roll_mode;

    _chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_LEN, MAVLINK_MSG_ID_GIMBAL_CONTROL_STANDARD_CRC);
#endif
}

