#ifndef _VFC_MAVLINK_H_
#define _VFC_MAVLINK_H_

//See https://mavlink.io/en/messages/common.html


struct MavLinkMessage_t{
    uint8_t magic;
    uint8_t len;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint8_t msgid;
    uint8_t payload[256];
    uint16_t checksum;
};

enum EMavlinkMessageType{
    SYS_STATUS = 1,
    SYSTEM_TIME = 2,
    PING = 4,
    CHANGE_OPERATOR_CONTROL = 5,
    CHANGE_OPERATOR_CONTROL_ACK = 6,
    SCALED_IMU = 26,
    RAW_IMU = 27,
    ATTITUDE = 30,
    OPTICAL_FLOW = 100,

}


#endif