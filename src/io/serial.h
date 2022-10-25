#include <stdarg.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../config/serial.h"
#include "../macros.h"


namespace serial
{
#define UCSRnA _CREATE_REG(UCSR, UART_INDEX, A)
#define UCSRnB _CREATE_REG(UCSR, UART_INDEX, B)
#define UCSRnC _CREATE_REG(UCSR, UART_INDEX, C)
#define UBRRnH _CREATE_REG(UBRR, UART_INDEX, H)
#define UBRRnL _CREATE_REG(UBRR, UART_INDEX, L)
#define UDRn _CREATE_REG(UDR, UART_INDEX, )

FORCE_INLINE void init()
{
    UCSRnA = _BV(U2X0);
    UCSRnB = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
    UCSRnC = _BV(UCSZ01) | _BV(UCSZ00);

    const uint16_t baudrate_divider = (((F_CPU + (4 * UART_BAUD)) / (8 * UART_BAUD)) - 1);
    UBRRnH = baudrate_divider >> 8;
    UBRRnL = baudrate_divider;
}

FORCE_INLINE void write(uint8_t c)
{
    while(!(UCSRnA & _BV(UDRE0)))
    {
    }
    UDRn = c;
}

FORCE_INLINE void write(const char* str)
{
    while(*str)
        write(*str++);
}

FORCE_INLINE void print(int32_t n)
{
    write(':');
    if (n < 0)
    {
        write('-');
        n = -n;
    }
    write('0' + ((n / 10000) % 10));
    write('0' + ((n / 1000) % 10));
    write('0' + ((n / 100) % 10));
    write('0' + ((n / 10) % 10));
    write('0' + (n % 10));
    write('\r');
    write('\n');
}

extern uint8_t receive_buffer[256];
extern volatile uint8_t receive_head, receive_tail;

FORCE_INLINE uint8_t read()
{
    if (receive_head != receive_tail)
    {
        uint8_t c = receive_buffer[receive_tail];
        receive_tail++;
        return c;
    }
    return 0;
}

FORCE_INLINE bool available()
{
    return receive_head != receive_tail;
}

};//namespace serial
