/**
  ******************************************************************************
  * @file    rf4463.c
  * @brief   RF4463 driver implementation - Direct port from Arduino library
  * @note    Follows Arduino RF4463.cpp structure exactly with debug prints
  ******************************************************************************
  */

#include "rf4463.h"
#include "radio_config_Si4463.h"
#include "debug_uart.h"  /* For myprintf debug output */

/* Include configuration data array */
const uint8_t RF4463_CONFIGURATION_DATA[] = RADIO_CONFIGURATION_DATA_ARRAY;

/* Private function prototypes */
static void RF4463_SPIInit(RF4463_HandleTypeDef *hrf);
static void RF4463_PinInit(RF4463_HandleTypeDef *hrf);
static void RF4463_PowerOnReset(RF4463_HandleTypeDef *hrf);
static void RF4463_EnterTxMode(RF4463_HandleTypeDef *hrf);
static void RF4463_EnterRxMode(RF4463_HandleTypeDef *hrf);
static void RF4463_SetConfig(RF4463_HandleTypeDef *hrf, const uint8_t *parameters, uint16_t paraLen);
static void RF4463_WriteTxFifo(RF4463_HandleTypeDef *hrf, uint8_t *databuf, uint8_t length);
static uint8_t RF4463_ReadRxFifo(RF4463_HandleTypeDef *hrf, uint8_t *databuf);
static void RF4463_FifoReset(RF4463_HandleTypeDef *hrf);
static bool RF4463_SetTxInterrupt(RF4463_HandleTypeDef *hrf);
static bool RF4463_SetRxInterrupt(RF4463_HandleTypeDef *hrf);
static bool RF4463_CheckCTS(RF4463_HandleTypeDef *hrf);
static void RF4463_SPIWriteBuf(RF4463_HandleTypeDef *hrf, uint8_t writeLen, uint8_t *writeBuf);
static void RF4463_SPIReadBuf(RF4463_HandleTypeDef *hrf, uint8_t readLen, uint8_t *readBuf);
static uint8_t RF4463_SPIByte(RF4463_HandleTypeDef *hrf, uint8_t writeData);

/**
  * @brief  Initialize SPI interface (if needed)
  * @note   In STM32, SPI is initialized by CubeMX, so this is mostly empty
  */
static void RF4463_SPIInit(RF4463_HandleTypeDef *hrf)
{
    /* SPI already initialized by CubeMX */
    /* Just ensure nSEL is HIGH (inactive) */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Initialize GPIO pins
  * @note   In STM32, pins are initialized by CubeMX, so this ensures proper states
  */
static void RF4463_PinInit(RF4463_HandleTypeDef *hrf)
{
    /* Set SDN HIGH (shutdown) */
    HAL_GPIO_WritePin(hrf->sdn_GPIO_Port, hrf->sdn_Pin, GPIO_PIN_SET);

    /* nIRQ is input - already configured by CubeMX */
}

/**
  * @brief  Initialize RF4463 module (with debug prints)
  * @note   Follows Arduino init() exactly
  */
bool RF4463_Init(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[20];

    myprintf("[RF4463] Inicializando RF4463...\r\n");

    /* Initialize pins and SPI */
    RF4463_PinInit(hrf);
    RF4463_SPIInit(hrf);

    /* Reset RF4463 */
    myprintf("[RF4463] Reseteando modulo...\r\n");
    RF4463_PowerOnReset(hrf);

    /* Set RF parameters from configuration file */
    myprintf("[RF4463] Cargando configuracion...\r\n");
    RF4463_SetConfig(hrf, RF4463_CONFIGURATION_DATA, sizeof(RF4463_CONFIGURATION_DATA));

    /* Set antenna switch (GPIO2 and GPIO3) */
    buf[0] = RF4463_GPIO_NO_CHANGE;
    buf[1] = RF4463_GPIO_NO_CHANGE;
    buf[2] = RF4463_GPIO_RX_STATE;
    buf[3] = RF4463_GPIO_TX_STATE;
    buf[4] = RF4463_NIRQ_INTERRUPT_SIGNAL;
    buf[5] = RF4463_GPIO_SPI_DATA_OUT;
    RF4463_SetCommand(hrf, 6, RF4463_CMD_GPIO_PIN_CFG, buf);

    /* Frequency adjust */
    buf[0] = 98;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_GLOBAL_XO_TUNE, 1, buf);

    /* TX = RX = 64 byte, PH mode, high performance mode */
    buf[0] = 0x40;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_GLOBAL_CONFIG, 1, buf);

    /* Set preamble */
    buf[0] = 0x08;  /* 8 bytes Preamble */
    buf[1] = 0x14;  /* Detect 20 bits */
    buf[2] = 0x00;
    buf[3] = 0x0F;
    buf[4] = RF4463_PREAMBLE_FIRST_1 | RF4463_PREAMBLE_LENGTH_BYTES | RF4463_PREAMBLE_STANDARD_1010;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    buf[8] = 0x00;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PREAMBLE_TX_LENGTH, 9, buf);

    /* Set sync words */
    buf[0] = 0x2D;
    buf[1] = 0xD4;
    RF4463_SetSyncWords(hrf, buf, 2);

    /* Set CRC */
    buf[0] = RF4463_CRC_SEED_ALL_1S | RF4463_CRC_ITU_T;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_CRC_CONFIG, 1, buf);

    buf[0] = RF4463_CRC_ENDIAN;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_CONFIG1, 1, buf);

    buf[0] = RF4463_IN_FIFO | RF4463_DST_FIELD_ENUM_2;
    buf[1] = RF4463_SRC_FIELD_ENUM_1;
    buf[2] = 0x00;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_LEN, 3, buf);

    /* Set length of Field 1 - 4 */
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = RF4463_FIELD_CONFIG_PN_START;
    buf[3] = RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
             RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE;
    buf[4] = 0x00;
    buf[5] = 50;
    buf[6] = RF4463_FIELD_CONFIG_PN_START;
    buf[7] = RF4463_FIELD_CONFIG_CRC_START | RF4463_FIELD_CONFIG_SEND_CRC |
             RF4463_FIELD_CONFIG_CHECK_CRC | RF4463_FIELD_CONFIG_CRC_ENABLE;
    buf[8] = 0x00;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = 0x00;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_FIELD_1_LENGTH_12_8, 12, buf);

    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_FIELD_4_LENGTH_12_8, 8, buf);

    /* Set max TX power */
    RF4463_SetTxPower(hrf, 127);
    myprintf("[RF4463] Potencia TX configurada al maximo\r\n");

    /* Check if RF4463 works */
    myprintf("[RF4463] Verificando dispositivo...\r\n");
    if (!RF4463_CheckDevice(hrf)) {
        myprintf("[RF4463] ERROR: Verificacion fallo!\r\n");
        return false;
    }

    myprintf("[RF4463] RF4463 OK!\r\n");
    return true;
}

/**
  * @brief  Power-on reset sequence (exactly like Arduino)
  */

static void RF4463_PowerOnReset(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[] = {RF_POWER_UP};

    myprintf("[POR] ===== Power-On Reset Start =====\r\n");

    /* Step 1: Put chip in shutdown */
    myprintf("[POR] Step 1: Setting SDN=HIGH (shutdown)...\r\n");
    HAL_GPIO_WritePin(hrf->sdn_GPIO_Port, hrf->sdn_Pin, GPIO_PIN_SET);
    myprintf("[POR] SDN state: %d (should be 1)\r\n",
             HAL_GPIO_ReadPin(hrf->sdn_GPIO_Port, hrf->sdn_Pin));
    HAL_Delay(100);

    /* Step 2: Release from shutdown */
    myprintf("[POR] Step 2: Setting SDN=LOW (active)...\r\n");
    HAL_GPIO_WritePin(hrf->sdn_GPIO_Port, hrf->sdn_Pin, GPIO_PIN_RESET);
    myprintf("[POR] SDN state: %d (should be 0)\r\n",
             HAL_GPIO_ReadPin(hrf->sdn_GPIO_Port, hrf->sdn_Pin));
    HAL_Delay(50);  // Increased from 20ms

    /* Step 3: Send power-up command */
    myprintf("[POR] Step 3: Sending POWER_UP command...\r\n");
    myprintf("[POR] Command bytes: ");
    for(int i = 0; i < sizeof(buf); i++) {
        myprintf("0x%02X ", buf[i]);
    }
    myprintf("\r\n");

    myprintf("[POR] NSEL before command: %d\r\n",
             HAL_GPIO_ReadPin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin));

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    myprintf("[POR] NSEL=LOW (chip selected)\r\n");

    RF4463_SPIWriteBuf(hrf, sizeof(buf), buf);

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);
    myprintf("[POR] NSEL=HIGH (chip deselected)\r\n");

    /* Step 4: Wait for boot */
    myprintf("[POR] Step 4: Waiting 200ms for chip boot...\r\n");
    HAL_Delay(500);

    myprintf("[POR] ===== Power-On Reset Complete =====\r\n");
    myprintf("[POR] Final pin states:\r\n");
    myprintf("[POR]   SDN:  %d (should be 0)\r\n",
             HAL_GPIO_ReadPin(hrf->sdn_GPIO_Port, hrf->sdn_Pin));
    myprintf("[POR]   NSEL: %d (should be 1)\r\n",
             HAL_GPIO_ReadPin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin));
    myprintf("[POR]   NIRQ: %d\r\n",
             HAL_GPIO_ReadPin(hrf->nIRQ_GPIO_Port, hrf->nIRQ_Pin));
    myprintf("\r\n");
}


/**
  * @brief  Set configuration from array (exactly like Arduino)
  */
static void RF4463_SetConfig(RF4463_HandleTypeDef *hrf, const uint8_t *parameters, uint16_t paraLen)
{
    uint8_t cmdLen;
    uint8_t command;
    uint16_t pos;
    uint8_t buf[30];

    /* Power up command already sent */
    paraLen = paraLen - 1;
    cmdLen = parameters[0];
    pos = cmdLen + 1;

    while (pos < paraLen) {
        cmdLen = parameters[pos++] - 1;  /* Get command length */
        command = parameters[pos++];      /* Get command */
        memcpy(buf, parameters + pos, cmdLen);  /* Get parameters */

        RF4463_SetCommand(hrf, cmdLen, command, buf);
        pos = pos + cmdLen;
    }
}

/**
  * @brief  Check if device is present and correct (with debug prints like Arduino)
  */
bool RF4463_CheckDevice(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[9];
    uint16_t partInfo;

    myprintf("[RF4463] Intentando getCommand...\r\n");

    if (!RF4463_GetCommand(hrf, 9, RF4463_CMD_PART_INFO, buf)) {
        myprintf("[RF4463] getCommand fallo\r\n");
        return false;
    }

    myprintf("[RF4463] Bytes recibidos: ");
    for (int i = 0; i < 9; i++) {
        myprintf("0x%02X ", buf[i]);
    }
    myprintf("\r\n");

    partInfo = (buf[2] << 8) | buf[3];
    myprintf("[RF4463] ID del dispositivo: 0x%04X\r\n", partInfo);

    if (partInfo != 0x4463) {
        myprintf("[RF4463] ID incorrecto\r\n");
        return false;
    }

    myprintf("[RF4463] Device check OK!\r\n");
    return true;
}

/**
  * @brief  Transmit packet (exactly like Arduino)
  */
bool RF4463_TxPacket(RF4463_HandleTypeDef *hrf, uint8_t *sendbuf, uint8_t sendLen)
{
    uint16_t txTimer;

    RF4463_FifoReset(hrf);              /* Clear FIFO */
    RF4463_WriteTxFifo(hrf, sendbuf, sendLen);  /* Load data to FIFO */
    RF4463_SetTxInterrupt(hrf);
    RF4463_ClrInterrupts(hrf);          /* Clear interrupt factors */
    RF4463_EnterTxMode(hrf);            /* Enter TX mode */

    txTimer = RF4463_TX_TIMEOUT;
    while (txTimer--) {
        if (RF4463_WaitIRQ(hrf)) {      /* Wait for interrupt */
            return true;
        }
        HAL_Delay(1);
    }
    RF4463_Init(hrf);  /* Reset RF4463 if TX timeout */

    return false;
}

/**
  * @brief  Receive packet (exactly like Arduino)
  */
uint8_t RF4463_RxPacket(RF4463_HandleTypeDef *hrf, uint8_t *recvbuf)
{
    uint8_t rxLen;
    rxLen = RF4463_ReadRxFifo(hrf, recvbuf);  /* Read data from FIFO */
    RF4463_FifoReset(hrf);                     /* Clear FIFO */

    return rxLen;
}

/**
  * @brief  Initialize RX mode (exactly like Arduino)
  */
bool RF4463_RxInit(RF4463_HandleTypeDef *hrf)
{
    uint8_t length = 50;
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length), &length);
    RF4463_FifoReset(hrf);      /* Clear FIFO */
    RF4463_SetRxInterrupt(hrf);
    RF4463_ClrInterrupts(hrf);  /* Clear interrupt factors */
    RF4463_EnterRxMode(hrf);    /* Enter RX mode */
    return true;
}

/**
  * @brief  Check for interrupt (exactly like Arduino)
  */
bool RF4463_WaitIRQ(RF4463_HandleTypeDef *hrf)
{
    /* Interrupt is active LOW */
    return (HAL_GPIO_ReadPin(hrf->nIRQ_GPIO_Port, hrf->nIRQ_Pin) == GPIO_PIN_RESET);
}

/**
  * @brief  Enter TX mode (exactly like Arduino)
  */
static void RF4463_EnterTxMode(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[] = {0x00, 0x30, 0x00, 0x00};
    buf[0] = RF4463_FREQ_CHANNEL;
    RF4463_SetCommand(hrf, 4, RF4463_CMD_START_TX, buf);
}

/**
  * @brief  Enter RX mode (exactly like Arduino)
  */
static void RF4463_EnterRxMode(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
    buf[0] = RF4463_FREQ_CHANNEL;
    RF4463_SetCommand(hrf, 7, RF4463_CMD_START_RX, buf);
}

/**
  * @brief  Enter standby mode (exactly like Arduino)
  */
bool RF4463_EnterStandbyMode(RF4463_HandleTypeDef *hrf)
{
    uint8_t data = 0x01;
    return RF4463_SetCommand(hrf, 1, RF4463_CMD_CHANGE_STATE, &data);
}

/**
  * @brief  Enable TX interrupt (exactly like Arduino)
  */
static bool RF4463_SetTxInterrupt(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[3] = {0x01, 0x20, 0x00};  /* Enable PACKET_SENT interrupt */
    return RF4463_SetProperties(hrf, RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

/**
  * @brief  Enable RX interrupt (exactly like Arduino)
  */
static bool RF4463_SetRxInterrupt(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[3] = {0x03, 0x18, 0x00};  /* Enable PACKET_RX interrupt */
    return RF4463_SetProperties(hrf, RF4463_PROPERTY_INT_CTL_ENABLE, 3, buf);
}

/**
  * @brief  Clear interrupts (exactly like Arduino)
  */
bool RF4463_ClrInterrupts(RF4463_HandleTypeDef *hrf)
{
    uint8_t buf[] = {0x00, 0x00, 0x00};
    return RF4463_SetCommand(hrf, sizeof(buf), RF4463_CMD_GET_INT_STATUS, buf);
}

/**
  * @brief  Write data to TX FIFO (exactly like Arduino)
  */
static void RF4463_WriteTxFifo(RF4463_HandleTypeDef *hrf, uint8_t *databuf, uint8_t length)
{
    RF4463_SetProperties(hrf, RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length), &length);
    uint8_t buf[length + 1];
    buf[0] = length;
    memcpy(buf + 1, databuf, length);
    RF4463_SetCommand(hrf, length + 1, RF4463_CMD_TX_FIFO_WRITE, buf);
}

/**
  * @brief  Read data from RX FIFO (exactly like Arduino)
  */
static uint8_t RF4463_ReadRxFifo(RF4463_HandleTypeDef *hrf, uint8_t *databuf)
{
    if (!RF4463_CheckCTS(hrf))
        return 0;

    uint8_t readLen;
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIByte(hrf, RF4463_CMD_RX_FIFO_READ);
    RF4463_SPIReadBuf(hrf, 1, &readLen);
    RF4463_SPIReadBuf(hrf, readLen, databuf);
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);
    return readLen;
}

/**
  * @brief  Reset FIFO (exactly like Arduino)
  */
static void RF4463_FifoReset(RF4463_HandleTypeDef *hrf)
{
    uint8_t data = 0x03;
    RF4463_SetCommand(hrf, sizeof(data), RF4463_CMD_FIFO_INFO, &data);
}

/**
  * @brief  Set GPIO mode (exactly like Arduino)
  */
bool RF4463_SetGPIOMode(RF4463_HandleTypeDef *hrf, uint8_t GPIO0Mode, uint8_t GPIO1Mode)
{
    uint8_t buf[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    buf[0] = GPIO0Mode;
    buf[1] = GPIO1Mode;

    return RF4463_SetCommand(hrf, sizeof(buf), RF4463_CMD_GPIO_PIN_CFG, buf);
}

/**
  * @brief  Set preamble length (exactly like Arduino)
  */
bool RF4463_SetPreambleLen(RF4463_HandleTypeDef *hrf, uint8_t len)
{
    return RF4463_SetProperties(hrf, RF4463_PROPERTY_PREAMBLE_TX_LENGTH, 1, &len);
}

/**
  * @brief  Set sync words (exactly like Arduino)
  */
bool RF4463_SetSyncWords(RF4463_HandleTypeDef *hrf, uint8_t *syncWords, uint8_t len)
{
    if ((len == 0) || (len > 3))
        return false;

    uint8_t buf[5];
    buf[0] = len - 1;
    memcpy(buf + 1, syncWords, len);
    return RF4463_SetProperties(hrf, RF4463_PROPERTY_SYNC_CONFIG, sizeof(buf), buf);
}

/**
  * @brief  Set TX power (exactly like Arduino)
  */
bool RF4463_SetTxPower(RF4463_HandleTypeDef *hrf, uint8_t power)
{
    if (power > 127)  /* Max is 127 */
        return false;

    uint8_t buf[4] = {0x08, 0x00, 0x00, 0x3D};
    buf[1] = power;

    return RF4463_SetProperties(hrf, RF4463_PROPERTY_PA_MODE, sizeof(buf), buf);
}

/**
  * @brief  Send command to RF4463 (exactly like Arduino)
  */
bool RF4463_SetCommand(RF4463_HandleTypeDef *hrf, uint8_t length, uint8_t command, uint8_t *paraBuf)
{
    if (!RF4463_CheckCTS(hrf))
        return false;

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIByte(hrf, command);           /* Send command */
    RF4463_SPIWriteBuf(hrf, length, paraBuf);  /* Send parameters */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    return true;
}

/**
  * @brief  Get command response from RF4463 (exactly like Arduino)
  */
bool RF4463_GetCommand(RF4463_HandleTypeDef *hrf, uint8_t length, uint8_t command, uint8_t *paraBuf)
{
    if (!RF4463_CheckCTS(hrf))
        return false;

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIByte(hrf, command);  /* Set command to read */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    if (!RF4463_CheckCTS(hrf))  /* Check if RF4463 is ready */
        return false;

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIByte(hrf, RF4463_CMD_READ_BUF);  /* Turn to read command mode */
    RF4463_SPIReadBuf(hrf, length, paraBuf);    /* Read parameters */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    return true;
}

/**
  * @brief  Set properties (exactly like Arduino)
  */
bool RF4463_SetProperties(RF4463_HandleTypeDef *hrf, uint16_t startProperty, uint8_t length, uint8_t *paraBuf)
{
    uint8_t buf[4];

    if (!RF4463_CheckCTS(hrf))
        return false;

    buf[0] = RF4463_CMD_SET_PROPERTY;
    buf[1] = startProperty >> 8;      /* GROUP */
    buf[2] = length;                  /* NUM_PROPS */
    buf[3] = startProperty & 0xFF;    /* START_PROP */

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIWriteBuf(hrf, 4, buf);            /* Set start property and read length */
    RF4463_SPIWriteBuf(hrf, length, paraBuf);   /* Set parameters */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    return true;
}

/**
  * @brief  Get properties (exactly like Arduino)
  */
bool RF4463_GetProperties(RF4463_HandleTypeDef *hrf, uint16_t startProperty, uint8_t length, uint8_t *paraBuf)
{
    if (!RF4463_CheckCTS(hrf))
        return false;

    uint8_t buf[4];
    buf[0] = RF4463_CMD_GET_PROPERTY;
    buf[1] = startProperty >> 8;      /* GROUP */
    buf[2] = length;                  /* NUM_PROPS */
    buf[3] = startProperty & 0xFF;    /* START_PROP */

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIWriteBuf(hrf, 4, buf);  /* Set start property and read length */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    if (!RF4463_CheckCTS(hrf))
        return false;

    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
    RF4463_SPIByte(hrf, RF4463_CMD_READ_BUF);  /* Turn to read command mode */
    RF4463_SPIReadBuf(hrf, length, paraBuf);    /* Read parameters */
    HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);

    return true;
}

/**
  * @brief  Check Clear-To-Send (exactly like Arduino)
  */
static bool RF4463_CheckCTS(RF4463_HandleTypeDef *hrf)
{
    uint16_t timeOutCnt = RF4463_CTS_TIMEOUT;

    while (timeOutCnt--) {  /* CTS counter */
        HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_RESET);
        RF4463_SPIByte(hrf, RF4463_CMD_READ_BUF);  /* Send READ_CMD_BUFF command */
        if (RF4463_SPIByte(hrf, 0) == RF4463_CTS_REPLY) {  /* Read CTS */
            HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);
            return true;
        }
        HAL_GPIO_WritePin(hrf->nSEL_GPIO_Port, hrf->nSEL_Pin, GPIO_PIN_SET);
    }
    return false;
}

/**
  * @brief  Write buffer via SPI (exactly like Arduino)
  */
static void RF4463_SPIWriteBuf(RF4463_HandleTypeDef *hrf, uint8_t writeLen, uint8_t *writeBuf)
{
    while (writeLen--)
        RF4463_SPIByte(hrf, *writeBuf++);
}

/**
  * @brief  Read buffer via SPI (exactly like Arduino)
  */
static void RF4463_SPIReadBuf(RF4463_HandleTypeDef *hrf, uint8_t readLen, uint8_t *readBuf)
{
    while (readLen--)
        *readBuf++ = RF4463_SPIByte(hrf, 0);
}

/**
  * @brief  Transfer single byte via SPI (exactly like Arduino)
  */
static uint8_t RF4463_SPIByte(RF4463_HandleTypeDef *hrf, uint8_t writeData)
{
    uint8_t readData;
    HAL_SPI_TransmitReceive(hrf->hspi, &writeData, &readData, 1, HAL_MAX_DELAY);
    return readData;
}
