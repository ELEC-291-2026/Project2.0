#ifndef DEBUG_UART_H
#define DEBUG_UART_H

void debug_uart_init(void);
void debug_uart_write_char(char character);
void debug_uart_write_string(const char *text);
void debug_uart_write_uint(unsigned int value);
void debug_uart_write_int(int value);

#endif
