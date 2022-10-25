#ifndef FAST_PWM_H
#define FAST_PWM_H

#include "fastio.h"

#define ___PWM_ENABLE(IO, TIMER, CHANNEL)  do { \
        SET_OUTPUT(IO); \
        WRITE(IO, 0); \
        TCCR ## TIMER ## A |= _BV( COM ## TIMER ## CHANNEL ## 1 ); \
        TCCR ## TIMER ## A |= _BV( WGM ## TIMER ## 0 ); \
        TCCR ## TIMER ## B = _BV(CS ## TIMER ## 0); \
        OCR ## TIMER ## CHANNEL = 0; \
    } while(0)

#define ___PWM_WRITE(IO, TIMER, CHANNEL, VALUE) do { \
        OCR ## TIMER ## CHANNEL = (VALUE); \
    } while(0)

#define __PWM_ENABLE(IO, TIMER, CHANNEL) ___PWM_ENABLE(IO, TIMER, CHANNEL)
#define __PWM_WRITE(IO, TIMER, CHANNEL, VALUE) ___PWM_WRITE(IO, TIMER, CHANNEL, VALUE)

#define _PWM_ENABLE(IO) __PWM_ENABLE(IO, PWMIO_ ## IO ## _TIMER, PWMIO_ ## IO ## _CHANNEL)
#define _PWM_WRITE(IO, VALUE) __PWM_WRITE(IO, PWMIO_ ## IO ## _TIMER, PWMIO_ ## IO ## _CHANNEL, VALUE)

#define PWM_ENABLE(IO) _PWM_ENABLE(IO)
#define PWM_WRITE(IO, VALUE) _PWM_WRITE(IO, VALUE)

#define PWMIO_E4_TIMER 3
#define PWMIO_E4_CHANNEL B
#define PWMIO_E5_TIMER 3
#define PWMIO_E5_CHANNEL C
#define PWMIO_G5_TIMER 0
#define PWMIO_G5_CHANNEL B
#define PWMIO_E3_TIMER 3
#define PWMIO_E3_CHANNEL A
#define PWMIO_H3_TIMER 4
#define PWMIO_H3_CHANNEL A
#define PWMIO_H4_TIMER 4
#define PWMIO_H4_CHANNEL B
#define PWMIO_H5_TIMER 4
#define PWMIO_H5_CHANNEL C
#define PWMIO_H6_TIMER 2
#define PWMIO_H6_CHANNEL B
#define PWMIO_B4_TIMER 2
#define PWMIO_B4_CHANNEL A
#define PWMIO_B5_TIMER 1
#define PWMIO_B5_CHANNEL A
#define PWMIO_B6_TIMER 1
#define PWMIO_B6_CHANNEL B
#define PWMIO_B7_TIMER 0
#define PWMIO_B7_CHANNEL A
#define PWMIO_L5_TIMER 5
#define PWMIO_L5_CHANNEL C
#define PWMIO_L4_TIMER 5
#define PWMIO_L4_CHANNEL B
#define PWMIO_L3_TIMER 5
#define PWMIO_L3_CHANNEL A

#endif//FAST_PWM_H
