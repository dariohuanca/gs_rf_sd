/* receiver_app.h - SIMPLIFIED VERSION */
#ifndef RECEIVER_APP_H
#define RECEIVER_APP_H

#include "stm32f7xx_hal.h"
#include "ax25_protocol.h"
#include "rf4463.h"
#include "fatfs.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Timing Constants */
#define BEACON_RESPONSE_DELAY 100
#define CONNECTION_TIMEOUT 180000  // 3 minutes

/* Commands from PC */
#define CMD_START_LINKING 0xC0
#define CMD_CAPTURE_PHOTO 0xC1
#define CMD_STATUS_REQUEST 0xC2

/* Receiver States */
typedef enum {
    IDLE,
    LINKING,
    LINKED,
    WAITING_SIZE,
    RECEIVING_IMAGE
} RxState;

/* Function Prototypes */
void Receiver_Init(RF4463_HandleTypeDef* radio_handle);
void Receiver_Loop(void);

#endif /* RECEIVER_APP_H */
