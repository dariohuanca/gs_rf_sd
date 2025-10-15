/* receiver_app.c */
#include "receiver_app.h"
#include "debug_uart.h"

extern UART_HandleTypeDef huart3;  // For PC communication

/**
 * @brief Initialize receiver context
 */
void Receiver_Init(ReceiverContext_t* ctx, RF4463_HandleTypeDef* hrf, GPIO_TypeDef* led_port, uint16_t led_pin) {
    memset(ctx, 0, sizeof(ReceiverContext_t));
    ctx->hrf = hrf;
    ctx->led_port = led_port;
    ctx->led_pin = led_pin;
    ctx->currentState = RX_STATE_IDLE;

    myprintf("[RX] Receiver initialized - Waiting for PC commands...\r\n");
}

/**
 * @brief Clear RF FIFO
 */
void Receiver_ClearFIFO(ReceiverContext_t* ctx) {
    RF4463_ClrInterrupts(ctx->hrf);
    uint8_t reset_cmd = 0x01;
    RF4463_SetCommand(ctx->hrf, 1, 0x15, &reset_cmd);
    HAL_Delay(20);
}

/**
 * @brief Send status message to PC via UART
 */
void Receiver_SendStatusToPC(const char* message) {
    uint8_t header[] = {0x8A, 0xDB};
    uint8_t msgLen = strlen(message);

    HAL_UART_Transmit(&huart3, header, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, &msgLen, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)message, msgLen, HAL_MAX_DELAY);
}

/**
 * @brief Send ACK to transmitter
 */
bool Receiver_SendAck(ReceiverContext_t* ctx) {
    Receiver_ClearFIFO(ctx);

    uint8_t temp_buffer[32];
    size_t pos = 0;

    // Destination callsign (TX01)
    uint8_t destCall[6] = {'T' << 1, 'X' << 1, '0' << 1, '1' << 1, ' ' << 1, ' ' << 1};
    uint8_t srcCall[6] = {'R' << 1, 'X' << 1, '0' << 1, '1' << 1, ' ' << 1, ' ' << 1};

    memcpy(&temp_buffer[pos], destCall, 6);
    pos += 6;
    temp_buffer[pos++] = 0x60;  // SSID

    memcpy(&temp_buffer[pos], srcCall, 6);
    pos += 6;
    temp_buffer[pos++] = 0x61;  // SSID + last address bit

    temp_buffer[pos++] = 0x03;  // Control
    temp_buffer[pos++] = AX25_PID_NONE;
    temp_buffer[pos++] = ACK;

    uint16_t crc = AX25_CalculateCRC16CCITT(temp_buffer, pos);
    temp_buffer[pos++] = crc & 0xFF;
    temp_buffer[pos++] = (crc >> 8) & 0xFF;

    uint8_t ack_frame[MAX_FRAME_SIZE];
    size_t total_size = 2 + pos;

    if (total_size > MAX_FRAME_SIZE) {
        myprintf("[RX] ERROR: ACK too large\r\n");
        return false;
    }

    ack_frame[0] = AX25_FLAG;
    memcpy(&ack_frame[1], temp_buffer, pos);
    ack_frame[1 + pos] = AX25_FLAG;

    HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_SET);
    bool success = RF4463_TxPacket(ctx->hrf, ack_frame, total_size);
    HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_RESET);

    if (success) {
        myprintf("[RX] ACK sent to TX\r\n");
    } else {
        myprintf("[RX] ACK failed\r\n");
    }

    HAL_Delay(5);
    RF4463_RxInit(ctx->hrf);

    return success;
}

/**
 * @brief Send NACK to transmitter
 */
bool Receiver_SendNack(ReceiverContext_t* ctx) {
    Receiver_ClearFIFO(ctx);

    uint8_t temp_buffer[32];
    size_t pos = 0;

    uint8_t destCall[6] = {'T' << 1, 'X' << 1, '0' << 1, '1' << 1, ' ' << 1, ' ' << 1};
    uint8_t srcCall[6] = {'R' << 1, 'X' << 1, '0' << 1, '1' << 1, ' ' << 1, ' ' << 1};

    memcpy(&temp_buffer[pos], destCall, 6);
    pos += 6;
    temp_buffer[pos++] = 0x60;

    memcpy(&temp_buffer[pos], srcCall, 6);
    pos += 6;
    temp_buffer[pos++] = 0x61;

    temp_buffer[pos++] = 0x03;
    temp_buffer[pos++] = AX25_PID_NONE;
    temp_buffer[pos++] = NACK;

    uint16_t crc = AX25_CalculateCRC16CCITT(temp_buffer, pos);
    temp_buffer[pos++] = crc & 0xFF;
    temp_buffer[pos++] = (crc >> 8) & 0xFF;

    uint8_t nack_frame[MAX_FRAME_SIZE];
    size_t total_size = 2 + pos;

    if (total_size > MAX_FRAME_SIZE) {
        myprintf("[RX] ERROR: NACK too large\r\n");
        return false;
    }

    nack_frame[0] = AX25_FLAG;
    memcpy(&nack_frame[1], temp_buffer, pos);
    nack_frame[1 + pos] = AX25_FLAG;

    HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_SET);
    bool success = RF4463_TxPacket(ctx->hrf, nack_frame, total_size);
    HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_RESET);

    if (!success) {
        myprintf("[RX] NACK failed\r\n");
    }

    HAL_Delay(20);
    RF4463_RxInit(ctx->hrf);

    return success;
}

/**
 * @brief Send beacon acknowledgment
 */
void Receiver_SendBeaconAck(ReceiverContext_t* ctx) {
    uint8_t ackData[] = {'A', 'C', 'K'};
    uint8_t frame[MAX_FRAME_SIZE];
    size_t frameLen = AX25_CreateFrame(frame, ackData, sizeof(ackData), PKT_TYPE_BEACON_ACK);

    RF4463_TxPacket(ctx->hrf, frame, frameLen);
    myprintf("[RX] Beacon ACK sent - LINKED\r\n");

    HAL_Delay(10);
    RF4463_RxInit(ctx->hrf);
}

/**
 * @brief Send photo capture command to transmitter
 */
void Receiver_SendPhotoCommand(ReceiverContext_t* ctx) {
    uint8_t cmdData[] = {'P', 'H', 'O', 'T', 'O'};
    uint8_t frame[MAX_FRAME_SIZE];
    size_t frameLen = AX25_CreateFrame(frame, cmdData, sizeof(cmdData), PKT_TYPE_CMD_PHOTO);

    RF4463_TxPacket(ctx->hrf, frame, frameLen);
    myprintf("[RX] Photo command sent to TX\r\n");
    Receiver_SendStatusToPC("CMD_PHOTO_SENT");

    HAL_Delay(10);
    RF4463_RxInit(ctx->hrf);
}

/**
 * @brief Generate unique filename for image
 */
static bool GenerateUniqueFilename(char* filename, size_t maxLen) {
    for (uint16_t i = 0; i < 10000; i++) {
        snprintf(filename, maxLen, "IMG_%04d.JPG", i);

        FILINFO fno;
        if (f_stat(filename, &fno) != FR_OK) {
            // File doesn't exist, use this name
            return true;
        }
    }
    return false;
}

/**
 * @brief Open new image file on SD card
 */
bool Receiver_OpenNewImageFile(ReceiverContext_t* ctx) {
    if (!GenerateUniqueFilename(ctx->currentImageFilename, sizeof(ctx->currentImageFilename))) {
        myprintf("[SD] Error generating filename\r\n");
        return false;
    }

    myprintf("[SD] Creating file: %s\r\n", ctx->currentImageFilename);

    FRESULT res = f_open(&ctx->imageFile, ctx->currentImageFilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        myprintf("[SD] Error creating file: %d\r\n", res);
        return false;
    }

    ctx->imageFileOpen = true;
    return true;
}

/**
 * @brief Close image file
 */
void Receiver_CloseImageFile(ReceiverContext_t* ctx) {
    if (ctx->imageFileOpen) {
        f_close(&ctx->imageFile);
        ctx->imageFileOpen = false;
        myprintf("[SD] File closed\r\n");
        myprintf("[SD] Image saved: %s\r\n", ctx->currentImageFilename);
    }
}

/**
 * @brief Check for commands from PC via UART
 */
void Receiver_CheckPcCommands(ReceiverContext_t* ctx) {
    uint8_t cmd;

    if (HAL_UART_Receive(&huart3, &cmd, 1, 0) == HAL_OK) {
        if (cmd == CMD_START_LINKING) {
            myprintf("[RX] LINK command from PC\r\n");

            if (ctx->currentState == RX_STATE_IDLE) {
                ctx->currentState = RX_STATE_LINKING;
                Receiver_SendStatusToPC("LINKING_STARTED");
                myprintf("[RX] LINKING mode active - Searching TX...\r\n");
                HAL_GPIO_TogglePin(ctx->led_port, ctx->led_pin);
            }
        }
        else if (cmd == CMD_CAPTURE_PHOTO) {
            myprintf("[RX] PHOTO command from PC\r\n");

            if (ctx->currentState == RX_STATE_LINKED) {
                Receiver_SendPhotoCommand(ctx);
                ctx->currentState = RX_STATE_WAITING_SIZE;
            } else {
                Receiver_SendStatusToPC("ERROR_NOT_LINKED");
                myprintf("[RX] Error: Not linked with TX\r\n");
            }
        }
        else if (cmd == CMD_STATUS_REQUEST) {
            if (ctx->currentState == RX_STATE_LINKED) {
                Receiver_SendStatusToPC("STATUS_LINKED");
            } else if (ctx->currentState == RX_STATE_LINKING) {
                Receiver_SendStatusToPC("STATUS_LINKING");
            } else {
                Receiver_SendStatusToPC("STATUS_IDLE");
            }
        }
        else if (cmd == ACK) {
            myprintf("[RX] PC confirmed data reception\r\n");
        }
    }
}

/**
 * @brief Main receiver processing function
 */
void Receiver_Process(ReceiverContext_t* ctx) {
    uint32_t currentTime = HAL_GetTick();

    // Check for connection timeout
    if (ctx->currentState == RX_STATE_LINKED ||
        ctx->currentState == RX_STATE_WAITING_SIZE ||
        ctx->currentState == RX_STATE_RECEIVING_IMAGE) {

        if (currentTime - ctx->lastTxActivityTime > CONNECTION_TIMEOUT) {
            myprintf("[RX] TIMEOUT - Link lost\r\n");
            Receiver_SendStatusToPC("LINK_LOST");

            Receiver_CloseImageFile(ctx);

            ctx->currentState = RX_STATE_IDLE;
            HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_RESET);
            ctx->expectedImageSize = 0;
            ctx->receivedBytes = 0;
        }
    }

    // Check for PC commands
    Receiver_CheckPcCommands(ctx);

    // Check for RF interrupt (incoming packet)
    if (RF4463_WaitIRQ(ctx->hrf)) {
        ctx->lastTxActivityTime = currentTime;
        ctx->packetsReceived++;

        RF4463_ClrInterrupts(ctx->hrf);

        uint8_t rx_buf[MAX_FRAME_SIZE];
        memset(rx_buf, 0, MAX_FRAME_SIZE);

        uint8_t rx_len = RF4463_RxPacket(ctx->hrf, rx_buf);

        if (rx_len > 0) {
            // Debug: show raw frame
            myprintf("\r\n[RX_RAW] Received %d bytes\r\n", rx_len);

            uint8_t parsed_data[MAX_PACKET_SIZE];
            size_t parsed_len;
            uint8_t packetType;

            if (AX25_ParseFrame(rx_buf, rx_len, parsed_data, &parsed_len, &packetType)) {
                myprintf("[RX] CRC VALID\r\n");
                ctx->packetsValid++;

                // Handle different packet types based on current state
                if (packetType == PKT_TYPE_BEACON && ctx->currentState == RX_STATE_LINKING) {
                    myprintf("[RX] Beacon detected from TX\r\n");
                    HAL_Delay(BEACON_RESPONSE_DELAY);
                    Receiver_SendBeaconAck(ctx);

                    ctx->currentState = RX_STATE_LINKED;
                    HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_SET);
                    Receiver_SendStatusToPC("SATELLITE_LINKED");
                    myprintf("[RX] *** LINKED WITH TX ***\r\n");
                }
                else if (packetType == PKT_TYPE_STATUS) {
                    parsed_data[parsed_len] = '\0';
                    myprintf("[RX] TX Status: %s\r\n", (char*)parsed_data);
                    Receiver_SendStatusToPC((char*)parsed_data);
                }
                else if (packetType == PKT_TYPE_CMD_ACK) {
                    myprintf("[RX] TX confirmed command\r\n");
                    Receiver_SendStatusToPC("TX_CMD_CONFIRMED");
                }
                else if (packetType == PKT_TYPE_IMG_SIZE && ctx->currentState == RX_STATE_WAITING_SIZE) {
                    if (parsed_len == 4) {
                        ctx->expectedImageSize = (parsed_data[0] << 24) | (parsed_data[1] << 16) |
                                                 (parsed_data[2] << 8) | parsed_data[3];

                        myprintf("[RX] Image size: %lu bytes\r\n", ctx->expectedImageSize);

                        if (ctx->expectedImageSize > 0 && ctx->expectedImageSize <= 100000) {
                            if (Receiver_OpenNewImageFile(ctx)) {
                                ctx->currentState = RX_STATE_RECEIVING_IMAGE;
                                ctx->receivedBytes = 0;

                                Receiver_SendAck(ctx);
                                myprintf("[RX] Size confirmed - SD file opened\r\n");
                            } else {
                                Receiver_SendNack(ctx);
                                myprintf("[RX] Error opening SD file\r\n");
                            }
                        } else {
                            Receiver_SendNack(ctx);
                            myprintf("[RX] Invalid size\r\n");
                        }
                    } else {
                        Receiver_SendNack(ctx);
                    }
                }
                else if (packetType == PKT_TYPE_IMG_DATA && ctx->currentState == RX_STATE_RECEIVING_IMAGE) {
                    if (ctx->imageFileOpen) {
                        myprintf("\r\n[RX_PACKET] %d bytes\r\n", parsed_len);

                        UINT bytesWritten;
                        FRESULT res = f_write(&ctx->imageFile, parsed_data, parsed_len, &bytesWritten);

                        if (res == FR_OK && bytesWritten == parsed_len) {
                            ctx->receivedBytes += parsed_len;

                            myprintf("[RX] Data: %dB (%lu%%)\r\n",
                                    parsed_len,
                                    (ctx->receivedBytes * 100) / ctx->expectedImageSize);

                            Receiver_SendAck(ctx);

                            if (ctx->receivedBytes >= ctx->expectedImageSize) {
                                Receiver_CloseImageFile(ctx);

                                myprintf("[RX] *** IMAGE COMPLETE ON SD ***\r\n");
                                Receiver_SendStatusToPC("IMAGE_COMPLETE");
                                ctx->currentState = RX_STATE_LINKED;
                                HAL_GPIO_WritePin(ctx->led_port, ctx->led_pin, GPIO_PIN_SET);
                            }
                        } else {
                            myprintf("[SD] Write error: %d\r\n", res);
                            Receiver_SendNack(ctx);
                        }
                    } else {
                        myprintf("[SD] Error: File not open\r\n");
                        Receiver_SendNack(ctx);
                    }
                }
                else if (packetType == PKT_TYPE_STATS) {
                    myprintf("[RX] Statistics received\r\n");
                }
            } else {
                myprintf("[RX] *** CRC INVALID ***\r\n");

                // Clear FIFO on error
                Receiver_ClearFIFO(ctx);
                HAL_Delay(10);
                RF4463_RxInit(ctx->hrf);

                Receiver_SendNack(ctx);
            }
        } else {
            myprintf("[RX_ERROR] Empty packet\r\n");
            Receiver_ClearFIFO(ctx);
            HAL_Delay(10);
            RF4463_RxInit(ctx->hrf);
        }
    }
}
