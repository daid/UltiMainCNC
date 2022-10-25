#include "src/config/pins.h"
#include "src/motion/planner.h"
#include "src/motion/stepper.h"
#include "src/io/fastio.h"
#include "src/io/fastpwm.h"
#include "src/io/serial.h"
#include "src/commandProcessor.h"

void setup() {
    SET_OUTPUT(POWER_ON_PIN);
    WRITE(POWER_ON_PIN, 1);

    serial::init();
    st_init();
    planner_init();
    st_enable_interrupt();

    sei();

    serial::write("start\r\n");
}

uint8_t serial_buffer_size;
char serial_buffer[80];
bool serial_blocking_mode = true;

void loop() {
    if (serial::available())
    {
        serial_buffer[serial_buffer_size] = serial::read();
        if (serial_buffer[serial_buffer_size] == '\r' || serial_buffer[serial_buffer_size] == '\n')
        {
            if (serial_buffer_size > 0)
            {
                serial_buffer[serial_buffer_size] = '\0';
                
                if (serial_buffer[0] == 'N')
                {
                    //Line line number and checksum, just strip it off for now.
                    char* c = strchr(serial_buffer, ' ');
                    if (c)
                    {
                        c++;
                        memmove(serial_buffer, c, strlen(c) + 1);
                        c = strchr(serial_buffer, '*');
                        if (c) *c = '\0';
                    }
                }
                serial::write(serial_buffer);
                serial::write("\r\n");
                
                Result result = Ok;
                do
                {
                    result = processCommand(serial_buffer);
                } while(serial_blocking_mode && result == Blocked);
                
                switch(result)
                {
                case Ok: serial::write("ok"); break;
                case Error: serial::write("error"); break;
                case Blocked: serial::write("blocked"); break;
                }
                serial::write("\r\n");
                serial_buffer_size = 0;
            }
        }
        else
        {
            serial_buffer_size++;
        }
    }
}
