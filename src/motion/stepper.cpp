/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "stepper.h"
#include "planner.h"
#include "speedLookupTable.h"
#include "../io/fastio.h"
#include "../io/fastpwm.h"
#include "../config/pins.h"

#include <avr/interrupt.h>

#if OUTPUT_AXIS_COUNT == 5
#define DO_LOOP(n) { n(0); n(1); n(2); n(3); n(4); }
#endif

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter[OUTPUT_AXIS_COUNT];       // Counter variables for the bresenham line tracer
static unsigned long step_events_completed; // The number of step events executed in the current block
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for decelaration start point
static uint8_t step_loops;
static uint8_t step_loops_nominal;
static unsigned short OCR1A_nominal;

#if defined(X_MIN_PIN) && X_MIN_PIN > -1
static bool old_x_min_endstop=false;
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1
static bool old_x_max_endstop=false;
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
static bool old_y_min_endstop=false;
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
static bool old_y_max_endstop=false;
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
static bool old_z_min_endstop=false;
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
static bool old_z_max_endstop=false;
#endif

static bool check_endstops = true;

//===========================================================================
//=============================functions         ============================
//===========================================================================

#ifdef __AVR
// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
    asm volatile ( \
    "clr r26 \n\t" \
    "mul %A1, %B2 \n\t" \
    "movw %A0, r0 \n\t" \
    "mul %A1, %A2 \n\t" \
    "add %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "lsr r0 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "clr r1 \n\t" \
    : \
    "=&r" (intRes) \
    : \
    "d" (charIn1), \
    "d" (intIn2) \
    : \
    "r26" \
    )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
    asm volatile ( \
    "clr r26 \n\t" \
    "mul %A1, %B2 \n\t" \
    "mov r27, r1 \n\t" \
    "mul %B1, %C2 \n\t" \
    "movw %A0, r0 \n\t" \
    "mul %C1, %C2 \n\t" \
    "add %B0, r0 \n\t" \
    "mul %C1, %B2 \n\t" \
    "add %A0, r0 \n\t" \
    "adc %B0, r1 \n\t" \
    "mul %A1, %C2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %B1, %B2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %C1, %A2 \n\t" \
    "add r27, r0 \n\t" \
    "adc %A0, r1 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %B1, %A2 \n\t" \
    "add r27, r1 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "lsr r27 \n\t" \
    "adc %A0, r26 \n\t" \
    "adc %B0, r26 \n\t" \
    "mul %D2, %A1 \n\t" \
    "add %A0, r0 \n\t" \
    "adc %B0, r1 \n\t" \
    "mul %D2, %B1 \n\t" \
    "add %B0, r0 \n\t" \
    "clr r1 \n\t" \
    : \
    "=&r" (intRes) \
    : \
    "d" (longIn1), \
    "d" (longIn2) \
    : \
    "r26" , "r27" \
    )
#else

// intRes = intIn1 * intIn2 >> 16
#define MultiU16X8toH16(intRes, charIn1, intIn2) do { (intRes) = (uint32_t(charIn1) * uint32_t(intIn2)) >> 16; } while(0)

// intRes = longIn1 * longIn2 >> 24
#define MultiU24X32toH16(intRes, longIn1, longIn2) do { (intRes) = (uint64_t(longIn1) * uint64_t(longIn2)) >> 24; } while(0)
#endif

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= _BV(OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &=~_BV(OCIE1A)

void st_enable_endstops(bool check)
{
    check_endstops = check;
}

void st_enable_motors()
{
    #define LOOP(n) \
        WRITE(MOTOR_ ## n ## _ENABLE_PIN, 0);
    DO_LOOP(LOOP);
    #undef LOOP
}

void st_disable_motors()
{
    #define LOOP(n) \
        WRITE(MOTOR_ ## n ## _ENABLE_PIN, 1);
    DO_LOOP(LOOP);
    #undef LOOP
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
FORCE_INLINE unsigned short calc_timer(unsigned short step_rate)
{
    unsigned short timer;
    if(step_rate > MAX_STEP_FREQUENCY)
        step_rate = MAX_STEP_FREQUENCY;

    if(step_rate > 20000) // If steprate > 20kHz >> step 4 times
    {
        step_rate = (step_rate >> 2)&0x3fff;
        step_loops = 4;
    }
    else if(step_rate > 10000) // If steprate > 10kHz >> step 2 times
    {
        step_rate = (step_rate >> 1)&0x7fff;
        step_loops = 2;
    }
    else
    {
        step_loops = 1;
    }

    if(step_rate < (F_CPU/500000))
        step_rate = (F_CPU/500000);
    step_rate -= (F_CPU/500000); // Correct for minimal speed
    if(step_rate >= (8*256)) // higher step rate
    {
        const uint8_t* table_address = (const uint8_t*)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
        uint8_t tmp_step_rate = (step_rate & 0x00ff);
        unsigned short gain = (unsigned short)pgm_read_word(table_address+2);
        MultiU16X8toH16(timer, tmp_step_rate, gain);
        timer = (unsigned short)pgm_read_word(table_address) - timer;
    }
    else
    {   // lower step rates
        const uint8_t* table_address = (const uint8_t*)&speed_lookuptable_slow[0][0];
        table_address += ((step_rate)>>1) & 0xfffc;
        timer = (unsigned short)pgm_read_word(table_address);
        timer -= (((unsigned short)pgm_read_word(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
    }
    if(timer < 100) //(20kHz this should never happen)
    {
        timer = 100;
    }
    return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset()
{
    deceleration_time = 0;
    // step_rate to timer interval
    OCR1A_nominal = calc_timer(current_block->nominal_rate);
    // make a note of the number of step loops required at nominal speed
    step_loops_nominal = step_loops;
    acc_step_rate = current_block->initial_rate;
    acceleration_time = calc_timer(acc_step_rate);
    OCR1A = acceleration_time;
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
// This function is timing critical, adding too much code/time/instructions in here will cause broken behaviour at high movement speeds.
// Do not touch this function unless you really know what you are doing.
ISR(TIMER1_COMPA_vect)
{
    // If there is no current block, attempt to pop one from the buffer
    if (current_block == NULL)
    {
        // Anything in the buffer?
        current_block = planner_get_current_block();
        if (current_block != NULL)
        {
            trapezoid_generator_reset();

            // Initialize Bresenham counters to 1/2 the ceiling
            counter[0] = -(current_block->step_event_count >> 1);
            for(uint8_t n=1; n<OUTPUT_AXIS_COUNT; n++)
                counter[n] = counter[0];
            step_events_completed = 0;
        }
        else
        {
            OCR1A=2000; // 1kHz.
        }
    }

    if (current_block != NULL)
    {
        // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
        out_bits = current_block->direction_bits;


        // Set the direction bits
        #define LOOP(n) \
            WRITE(MOTOR_ ## n ## _DIR_PIN, out_bits & _BV(n));
        DO_LOOP(LOOP);
        #undef LOOP

        // Set direction en check limit switches
        if (check_endstops)
        {
            if (out_bits & _BV(0))
            {
                #if defined(X_MIN_PIN) && X_MIN_PIN > -1
                bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
                if(x_min_endstop && old_x_min_endstop && (current_block->steps[0] > 0))
                    step_events_completed = current_block->step_event_count;
                old_x_min_endstop = x_min_endstop;
                #endif
            }
            else
            { // +direction
                #if defined(X_MAX_PIN) && X_MAX_PIN > -1
                bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
                if(x_max_endstop && old_x_min_endstop && (current_block->steps[0] > 0))
                    step_events_completed = current_block->step_event_count;
                old_x_max_endstop = x_max_endstop;
                #endif
            }
        }

        for(int8_t i=0; i < step_loops; i++)  // Take multiple steps per interrupt (For high speed moves)
        {
            //TODO: MSerial.checkRx(); // Check for serial chars.

            #define LOOP(n) \
                counter[n] += current_block->steps[n]; \
                if (counter[n] > 0) { \
                    WRITE(MOTOR_ ## n ## _STEP_PIN, 1); \
                    counter[n] -= current_block->step_event_count; \
                    WRITE(MOTOR_ ## n ## _STEP_PIN, 0); \
                }
            DO_LOOP(LOOP);
            #undef LOOP
            
            step_events_completed += 1;
            if(step_events_completed >= current_block->step_event_count)
                break;
        }
        // Calculate new timer value
        if (step_events_completed <= (uint32_t)current_block->accelerate_until)
        {
            MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
            acc_step_rate += current_block->initial_rate;

            // upper limit
            if(acc_step_rate > current_block->nominal_rate)
                acc_step_rate = current_block->nominal_rate;

            // step_rate to timer interval
            uint16_t timer = calc_timer(acc_step_rate);
            OCR1A = timer;
            acceleration_time += timer;
        }
        else if (step_events_completed > (uint32_t)current_block->decelerate_after)
        {
            uint16_t step_rate;
            MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

            if (step_rate < acc_step_rate)
            { // Still decelerating?
                step_rate = MAX(acc_step_rate - step_rate, current_block->final_rate);
            }
            else
            {
                step_rate = current_block->final_rate;  // lower limit
            }

            // step_rate to timer interval
            uint16_t timer = calc_timer(step_rate);
            OCR1A = timer;
            deceleration_time += timer;
        }
        else
        {
            OCR1A = OCR1A_nominal;
            // ensure we're running at the correct step rate, even if we just came off an acceleration
            step_loops = step_loops_nominal;
        }

        // Hack to address stuttering caused by ISR not finishing in time.
        // When the ISR does not finish in time, the timer will wrap in the computation of the next interrupt time.
        // This hack replaces the correct (past) time with a time not far in the future.
        // (Note that OCR1A and TCNT1 are registers, so using the max() macro or std::max() can cause problems, especially when compiling the simulator)
        if (OCR1A < TCNT1 + 16)
            OCR1A = TCNT1 + 16;

        // If current block is finished, reset pointer
        if (step_events_completed >= current_block->step_event_count)
        {
            current_block = NULL;
            planner_discard_current_block();
        }
    }
}

void st_init()
{
    PWM_ENABLE(MOTOR_CURRENT_PWM_XY_PIN);
    PWM_ENABLE(MOTOR_CURRENT_PWM_Z_PIN);
    PWM_ENABLE(MOTOR_CURRENT_PWM_E_PIN);
    PWM_WRITE(MOTOR_CURRENT_PWM_XY_PIN, 1300L * 255L / MOTOR_CURRENT_PWM_RANGE);
    PWM_WRITE(MOTOR_CURRENT_PWM_Z_PIN, 1300L * 255L / MOTOR_CURRENT_PWM_RANGE);
    PWM_WRITE(MOTOR_CURRENT_PWM_E_PIN, 1300L * 255L / MOTOR_CURRENT_PWM_RANGE);

    #define LOOP(n) \
        SET_OUTPUT(MOTOR_ ## n ## _STEP_PIN); \
        SET_OUTPUT(MOTOR_ ## n ## _DIR_PIN); \
        SET_OUTPUT(MOTOR_ ## n ## _ENABLE_PIN); \
        WRITE(MOTOR_ ## n ## _ENABLE_PIN, 1);
    DO_LOOP(LOOP);
    #undef LOOP

    //endstops and pullups
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
    #endif

    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
    #endif

    #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
    #endif

    #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
    #endif

    #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
    #endif

    #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
    #endif

    st_enable_endstops(true); // Start with endstops active. After homing they can be disabled
}

void st_quickStop()
{
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    while(blocks_queued())
    {
        //TODO: Adjust "final_step_position" with discarded blocks so planner is in sync.
        planner_discard_current_block();
    }
    current_block = NULL;
    ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// Enable the handling of the interrupts
void st_enable_interrupt()
{
    // waveform generation = 0100 = CTC
    TCCR1B &= ~(1<<WGM13);
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~(1<<WGM11);
    TCCR1A &= ~(1<<WGM10);

    // output mode = 00 (disconnected)
    TCCR1A &= ~(3<<COM1A0);
    TCCR1A &= ~(3<<COM1B0);

    // Set the timer pre-scaler
    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU. If you are going to change this, be
    // sure to regenerate speed_lookuptable.h with
    // create_speed_lookuptable.py
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

    // Init Stepper ISR to 122 Hz for quick starting
    OCR1A = 0x4000;
    TCNT1 = 0;
    ENABLE_STEPPER_DRIVER_INTERRUPT();
}
