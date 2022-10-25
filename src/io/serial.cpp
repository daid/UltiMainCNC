#include "serial.h"

namespace serial
{
uint8_t receive_buffer[256];
volatile uint8_t receive_head, receive_tail;
};//namespace serial


ISR(_CREATE_REG(USART, UART_INDEX, _RX_vect))
{
    uint8_t n = serial::receive_head + 1;
    if (n != serial::receive_tail)
    {
        serial::receive_buffer[serial::receive_head] = UDRn;
        serial::receive_head = n;
    }
}
