/* receiver_app.c - SIMPLIFIED VERSION (like Arduino) */
#include "receiver_app.h"
#include "debug_uart.h"

extern UART_HandleTypeDef huart3;  // For PC communication

// ========== GLOBAL VARIABLES (like Arduino) ==========
RF4463_HandleTypeDef* radio;

RxState currentState = IDLE;

uint32_t expectedImageSize = 0;
uint32_t receivedBytes = 0;
uint32_t packetsReceived = 0;
uint32_t packetsValid = 0;

uint32_t lastTxActivityTime = 0;

char currentImageFilename[32];
bool imageFileOpen = false;
FIL imageFile;

// ======================================================

void clearFIFO(void) {
    RF4463_ClrInterrupts(radio);
    uint8_t reset_cmd = 0x01;
    RF4463_SetCommand(radio, 1, 0x15, &reset_cmd);
    HAL_Delay(20);
}

void sendStatusToPC(const char* message) {
    uint8_t header[] = {0x8A, 0xDB};
    uint8_t msgLen = strlen(message);

    HAL_UART_Transmit(&huart3, header, 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, &msgLen, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)message, msgLen, HAL_MAX_DELAY);
}

bool sendAck(void) {
    clearFIFO();

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

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    bool success = RF4463_TxPacket(radio, ack_frame, total_size);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

    if (success) {
        myprintf("[RX] ACK sent to TX\r\n");
    } else {
        myprintf("[RX] ACK failed\r\n");
    }

    HAL_Delay(5);
    RF4463_RxInit(radio);

    return success;
}

bool sendNack(void) {
    clearFIFO();

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

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    bool success = RF4463_TxPacket(radio, nack_frame, total_size);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

    if (!success) {
        myprintf("[RX] NACK failed\r\n");
    }

    HAL_Delay(20);
    RF4463_RxInit(radio);

    return success;
}

void sendBeaconAck(void) {
    uint8_t ackData[] = {'A', 'C', 'K'};
    uint8_t frame[MAX_FRAME_SIZE];
    size_t frameLen = AX25_CreateFrame(frame, ackData, sizeof(ackData), PKT_TYPE_BEACON_ACK);

    RF4463_TxPacket(radio, frame, frameLen);
    myprintf("[RX] Beacon ACK enviado - ENLAZADO\r\n");

    HAL_Delay(10);
    RF4463_RxInit(radio);
}

void sendPhotoCommand(void) {
    uint8_t cmdData[] = {'P', 'H', 'O', 'T', 'O'};
    uint8_t frame[MAX_FRAME_SIZE];
    size_t frameLen = AX25_CreateFrame(frame, cmdData, sizeof(cmdData), PKT_TYPE_CMD_PHOTO);

    RF4463_TxPacket(radio, frame, frameLen);
    myprintf("[RX] Comando PHOTO enviado al TX\r\n");
    sendStatusToPC("CMD_PHOTO_SENT");

    HAL_Delay(10);
    RF4463_RxInit(radio);
}

bool openNewImageFile(void) {
    // Generate unique filename
    for (uint16_t i = 0; i < 10000; i++) {
        snprintf(currentImageFilename, sizeof(currentImageFilename), "IMG_20250101_%06d.JPG", i);

        FILINFO fno;
        if (f_stat(currentImageFilename, &fno) != FR_OK) {
            // File doesn't exist, use this name
            break;
        }
    }

    myprintf("[SD] Creando archivo: %s\r\n", currentImageFilename);

    FRESULT res = f_open(&imageFile, currentImageFilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        myprintf("[SD] Error al crear el archivo %s\r\n", currentImageFilename);
        return false;
    }

    imageFileOpen = true;
    return true;
}

void closeImageFile(void) {
    if (imageFileOpen) {
        f_close(&imageFile);
        imageFileOpen = false;
        myprintf("[SD] Archivo cerrado\r\n");
        myprintf("[SD] Imagen guardada: %s\r\n", currentImageFilename);
    }
}

void checkPcCommands(void) {
    uint8_t cmd;

    if (HAL_UART_Receive(&huart3, &cmd, 1, 0) == HAL_OK) {
        if (cmd == CMD_START_LINKING) {
            myprintf("[RX] Comando ENLAZAR recibido desde PC\r\n");

            if (currentState == IDLE) {
                currentState = LINKING;
                sendStatusToPC("LINKING_STARTED");
                myprintf("[RX] Modo LINKING activado - Buscando TX...\r\n");
                HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            }
        }
        else if (cmd == CMD_CAPTURE_PHOTO) {
            myprintf("[RX] Comando CAPTURA_FOTO recibido desde PC\r\n");

            if (currentState == LINKED) {
                sendPhotoCommand();
                currentState = WAITING_SIZE;
            } else {
                sendStatusToPC("ERROR_NOT_LINKED");
                myprintf("[RX] Error: No enlazado con TX\r\n");
            }
        }
        else if (cmd == CMD_STATUS_REQUEST) {
            if (currentState == LINKED) {
                sendStatusToPC("STATUS_LINKED");
            } else if (currentState == LINKING) {
                sendStatusToPC("STATUS_LINKING");
            } else {
                sendStatusToPC("STATUS_IDLE");
            }
        }
        else if (cmd == ACK) {
            myprintf("[RX] PC confirmo recepcion de datos\r\n");
        }
    }
}

void Receiver_Init(RF4463_HandleTypeDef* radio_handle) {
    radio = radio_handle;
    currentState = IDLE;

    myprintf("[RX] Radio OK - Esperando comandos PC...\r\n");
}

void Receiver_Loop(void) {
    uint32_t currentTime = HAL_GetTick();

    // Check for connection timeout
    if (currentState == LINKED || currentState == WAITING_SIZE || currentState == RECEIVING_IMAGE) {
        if (currentTime - lastTxActivityTime > CONNECTION_TIMEOUT) {
            myprintf("[RX] TIMEOUT - Enlace perdido\r\n");
            sendStatusToPC("LINK_LOST");

            closeImageFile();

            currentState = IDLE;
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
            expectedImageSize = 0;
            receivedBytes = 0;
        }
    }

    // Check for PC commands
    checkPcCommands();

    // Check for RF interrupt (incoming packet)
    if (RF4463_WaitIRQ(radio)) {
        lastTxActivityTime = currentTime;
        packetsReceived++;

        RF4463_ClrInterrupts(radio);

        uint8_t rx_buf[MAX_FRAME_SIZE];
        memset(rx_buf, 0, MAX_FRAME_SIZE);

        uint8_t rx_len = RF4463_RxPacket(radio, rx_buf);

        if (rx_len > 0) {
            // ========== DEBUG: SHOW RAW FRAME ==========
            myprintf("\r\n[RX_RAW_FRAME] Recibido frame de %d bytes:\r\n", rx_len);
            myprintf("[RX_RAW_HEX] ");
            for (uint8_t i = 0; i < rx_len; i++) {
                if (rx_buf[i] < 0x10) myprintf("0");
                myprintf("%02X ", rx_buf[i]);
                if ((i + 1) % 16 == 0) myprintf("\r\n");
            }
            myprintf("\r\n");
            // ===========================================

            uint8_t parsed_data[MAX_PACKET_SIZE];
            size_t parsed_len;
            uint8_t packetType;

            if (AX25_ParseFrame(rx_buf, rx_len, parsed_data, &parsed_len, &packetType)) {
                myprintf("[RX_PARSE] *** CRC VALIDO ***\r\n");
                packetsValid++;

                // Handle different packet types based on current state
                if (packetType == PKT_TYPE_BEACON && currentState == LINKING) {
                    myprintf("[RX] Beacon detectado del TX\r\n");
                    HAL_Delay(BEACON_RESPONSE_DELAY);
                    sendBeaconAck();

                    currentState = LINKED;
                    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
                    sendStatusToPC("SATELLITE_LINKED");
                    myprintf("[RX] *** ENLAZADO CON TX ***\r\n");
                }
                else if (packetType == PKT_TYPE_STATUS) {
                    parsed_data[parsed_len] = '\0';
                    myprintf("[RX] TX Status: %s\r\n", (char*)parsed_data);
                    sendStatusToPC((char*)parsed_data);
                }
                else if (packetType == PKT_TYPE_CMD_ACK) {
                    myprintf("[RX] TX confirmo comando\r\n");
                    sendStatusToPC("TX_CMD_CONFIRMED");
                }
                else if (packetType == PKT_TYPE_IMG_SIZE && currentState == WAITING_SIZE) {
                    if (parsed_len == 4) {
                        expectedImageSize = (parsed_data[0] << 24) | (parsed_data[1] << 16) |
                                           (parsed_data[2] << 8) | parsed_data[3];

                        myprintf("[RX] Tamano imagen: %lu bytes\r\n", expectedImageSize);

                        if (expectedImageSize > 0 && expectedImageSize <= 100000) {
                            if (openNewImageFile()) {
                                currentState = RECEIVING_IMAGE;
                                receivedBytes = 0;

                                sendAck();
                                myprintf("[RX] Tamano confirmado - Archivo SD abierto\r\n");
                            } else {
                                sendNack();
                                myprintf("[RX] Error al abrir archivo SD\r\n");
                            }
                        } else {
                            sendNack();
                            myprintf("[RX] Tamano invalido\r\n");
                        }
                    } else {
                        sendNack();
                    }
                }
                else if (packetType == PKT_TYPE_IMG_DATA && currentState == RECEIVING_IMAGE) {
                    if (imageFileOpen) {
                        // ========== DEBUG: SHOW RECEIVED BYTES ==========
                        myprintf("\r\n[RX_PACKET_RECEIVED] Bytes: %d\r\n", parsed_len);

                        myprintf("[RX_DATA_HEX] ");
                        for (size_t i = 0; i < parsed_len; i++) {
                            if (parsed_data[i] < 0x10) myprintf("0");
                            myprintf("%02X ", parsed_data[i]);
                            if ((i + 1) % 16 == 0) myprintf("\r\n");
                        }
                        myprintf("\r\n");

                        myprintf("[RX_DATA_DEC] Primeros 8 bytes: ");
                        for (size_t i = 0; i < (parsed_len < 8 ? parsed_len : 8); i++) {
                            myprintf("%d ", parsed_data[i]);
                        }
                        myprintf("\r\n");
                        // ================================================

                        UINT bytesWritten;
                        FRESULT res = f_write(&imageFile, parsed_data, parsed_len, &bytesWritten);

                        if (res == FR_OK && bytesWritten == parsed_len) {
                            receivedBytes += parsed_len;

                            myprintf("[RX] Datos: %dB (%lu%%)\r\n",
                                    parsed_len,
                                    (receivedBytes * 100) / expectedImageSize);

                            sendAck();

                            if (receivedBytes >= expectedImageSize) {
                                closeImageFile();

                                myprintf("[RX] *** IMAGEN COMPLETA EN SD ***\r\n");
                                sendStatusToPC("IMAGE_COMPLETE");
                                currentState = LINKED;
                                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
                            }
                        } else {
                            myprintf("[SD] Error de escritura en SD\r\n");
                            sendNack();
                        }
                    } else {
                        myprintf("[SD] Error: Archivo no abierto\r\n");
                        sendNack();
                    }
                }
                else if (packetType == PKT_TYPE_STATS) {
                    myprintf("[RX] Estadisticas recibidas\r\n");
                }
            } else {
                myprintf("[RX] *** CRC INVALIDO ***\r\n");
                myprintf("[RX_CRC_FAIL] Frame len=%d, FLAG_START=0x%02X, FLAG_END=0x%02X\r\n",
                        rx_len, rx_buf[0], rx_buf[rx_len - 1]);

                if (rx_len >= 19) {
                    myprintf("[RX_CRC_FAIL] Control=0x%02X, PID=0x%02X\r\n",
                            rx_buf[15], rx_buf[16]);

                    myprintf("[RX_CRC_FAIL] CRC recibido: 0x%02X 0x%02X\r\n",
                            rx_buf[rx_len - 3], rx_buf[rx_len - 2]);
                }

                clearFIFO();
                HAL_Delay(10);
                RF4463_RxInit(radio);

                sendNack();
            }
        } else {
            myprintf("[RX_ERROR] Empty packet\r\n");
            clearFIFO();
            HAL_Delay(10);
            RF4463_RxInit(radio);
        }
    }
}
