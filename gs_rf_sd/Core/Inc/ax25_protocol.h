/* ax25_protocol.h */
#ifndef AX25_PROTOCOL_H
#define AX25_PROTOCOL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* AX.25 Protocol Constants */
#define MAX_FRAME_SIZE 64
#define MAX_PACKET_SIZE 30

/* AX.25 Frame Constants */
#define AX25_FLAG 0x7E
#define AX25_PID_NONE 0xF0

/* Packet Types */
#define PKT_TYPE_BEACON 0x01
#define PKT_TYPE_BEACON_ACK 0x02
#define PKT_TYPE_CMD_PHOTO 0x03
#define PKT_TYPE_CMD_ACK 0x04
#define PKT_TYPE_STATUS 0x05
#define PKT_TYPE_IMG_SIZE 0x10
#define PKT_TYPE_IMG_DATA 0x11
#define PKT_TYPE_STATS 0x20

/* ACK/NACK Codes */
#define ACK 0xAC
#define NACK 0x15

/* Callsigns */
#define SOURCE_CALLSIGN "RX01"
#define DEST_CALLSIGN "TX01"
#define SOURCE_SSID 0
#define DEST_SSID 0

/* Function Prototypes */
uint16_t AX25_CalculateCRC16CCITT(const uint8_t* data, size_t len);
void AX25_EncodeAddress(uint8_t* output, const char* callsign, uint8_t ssid, bool isLast);
size_t AX25_CreateFrame(uint8_t* frame, const uint8_t* data, size_t dataLen, uint8_t packetType);
bool AX25_ParseFrame(const uint8_t* frame, size_t frameLen, uint8_t* data, size_t* dataLen, uint8_t* packetType);

#endif /* AX25_PROTOCOL_H */
