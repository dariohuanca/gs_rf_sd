/* receiver_app.h */
#ifndef RECEIVER_APP_H
#define RECEIVER_APP_H

#include "stm32f4xx_hal.h"
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
    RX_STATE_IDLE,
    RX_STATE_LINKING,
    RX_STATE_LINKED,
    RX_STATE_WAITING_SIZE,
    RX_STATE_RECEIVING_IMAGE
} RxState_t;

/* Receiver Context Structure */
typedef struct {
    RxState_t currentState;

    uint32_t expectedImageSize;
    uint32_t receivedBytes;
    uint32_t packetsReceived;
    uint32_t packetsValid;

    uint32_t lastTxActivityTime;

    char currentImageFilename[32];
    bool imageFileOpen;
    FIL imageFile;

    RF4463_HandleTypeDef* hrf;
    GPIO_TypeDef* led_port;
    uint16_t led_pin;

} ReceiverContext_t;

/* Function Prototypes */
void Receiver_Init(ReceiverContext_t* ctx, RF4463_HandleTypeDef* hrf, GPIO_TypeDef* led_port, uint16_t led_pin);
void Receiver_Process(ReceiverContext_t* ctx);
void Receiver_CheckPcCommands(ReceiverContext_t* ctx);
bool Receiver_SendAck(ReceiverContext_t* ctx);
bool Receiver_SendNack(ReceiverContext_t* ctx);
void Receiver_SendBeaconAck(ReceiverContext_t* ctx);
void Receiver_SendPhotoCommand(ReceiverContext_t* ctx);
void Receiver_SendStatusToPC(const char* message);
bool Receiver_OpenNewImageFile(ReceiverContext_t* ctx);
void Receiver_CloseImageFile(ReceiverContext_t* ctx);
void Receiver_ClearFIFO(ReceiverContext_t* ctx);

#endif /* RECEIVER_APP_H */
