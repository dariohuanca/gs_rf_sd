/**
  ******************************************************************************
  * @file    debug_uart.c
  * @brief   UART debug print utility implementation
  ******************************************************************************
  */

#include "debug_uart.h"

#ifdef DEBUG_UART_ENABLE

/* External UART handle - must be defined in main.c */
extern UART_HandleTypeDef DEBUG_UART_HANDLE;

/* Runtime enable/disable flag */
static bool debug_enabled = true;  // Default: enabled

/**
  * @brief  Enable or disable debug prints at runtime
  * @param  enable: true to enable, false to disable
  * @retval None
  */
void debug_uart_set_enable(bool enable) {
    debug_enabled = enable;
}

/**
  * @brief  Check if debug is enabled
  * @retval true if enabled, false otherwise
  */
bool debug_uart_is_enabled(void) {
    return debug_enabled;
}

/**
  * @brief  Printf-like function over UART (only if enabled)
  * @param  fmt: Format string (same as printf)
  * @param  ...: Variable arguments
  * @retval None
  */
void myprintf(const char *fmt, ...) {
    if (!debug_enabled) {
        return;  // Debug is disabled, do nothing
    }

    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

#endif /* DEBUG_UART_ENABLE */
