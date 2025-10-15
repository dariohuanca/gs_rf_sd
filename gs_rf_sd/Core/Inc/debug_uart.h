/**
  ******************************************************************************
  * @file    debug_uart.h
  * @brief   UART debug print utility with enable/disable control
  ******************************************************************************
  */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ============================================
 * COMPILE-TIME DEBUG CONTROL
 * Uncomment the line below to enable debug prints
 * Comment it out to completely disable (zero overhead)
 * ============================================ */
#define DEBUG_UART_ENABLE

/* ============================================
 * RUNTIME DEBUG CONTROL
 * Use debug_uart_set_enable() to enable/disable at runtime
 * ============================================ */

#ifdef DEBUG_UART_ENABLE
    /* Debug is compiled in - can be controlled at runtime */
    void myprintf(const char *fmt, ...);
    void debug_uart_set_enable(bool enable);
    bool debug_uart_is_enabled(void);
#else
    /* Debug is completely disabled - no code generated */
    #define myprintf(fmt, ...) ((void)0)
    #define debug_uart_set_enable(enable) ((void)0)
    #define debug_uart_is_enabled() (false)
#endif

#endif /* DEBUG_UART_H */
