#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define CH32V003 1
#define FUNCONF_SYSTICK_USE_HCLK 1

// printf goes out real UART1 (TX=PD5, RX=PD6), read via the WCH-LinkE's
// own RXD/TXD pins wired to a USB-serial port (e.g. /dev/ttyACM0) at
// 115200 baud. Commands are read back by polling USART1 RX directly.
#define FUNCONF_USE_DEBUGPRINTF 0
#define FUNCONF_USE_UARTPRINTF 1
#define FUNCONF_UART_PRINTF_BAUD 115200

#endif
