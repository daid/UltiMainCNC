/*
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef	FAST_IO_H
#define	FAST_IO_H

#include <avr/io.h>
#include <util/atomic.h>

/*
  utility functions
*/

#ifndef MASK
/// MASKING- returns \f$2^PIN\f$
#define MASK(PIN)  (1 << PIN)
#endif

/*
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))
/// write to a pin
// On some boards pins > 0x100 are used (for example port H, J, K and L on AtMega 2560).
// These are not converted to atomic actions. A critical section is needed.

#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do { if (v) { \
                                         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) \
                                         {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); }\
                                       }\
                                       else {\
                                         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) \
                                         {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }\
                                       }\
                                     }\
                                     while (0)

#define _WRITE(IO, v)  do {  if ((int)&(DIO ##  IO ## _RPORT) >= 0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

/// set pin as input
#define	_SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
/// set pin as output
#define	_SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)

/// check if pin is an input
#define	_GET_INPUT(IO)  ((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define	_GET_OUTPUT(IO)  ((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE(IO, v)  _WRITE(IO, v)

/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// set pin as output wrapper
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define GET_INPUT(IO)  _GET_INPUT(IO)
/// check if pin is an output wrapper
#define GET_OUTPUT(IO)  _GET_OUTPUT(IO)

/*
	ports and functions

	added as necessary or if I feel like it- not a comprehensive list!
*/

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
/*
pins
*/
#define	DIO0_PIN		PINE0
#define	DIO0_RPORT	PINE
#define	DIO0_WPORT	PORTE
#define	DIO0_DDR		DDRE

#define	DIO1_PIN		PINE1
#define	DIO1_RPORT	PINE
#define	DIO1_WPORT	PORTE
#define	DIO1_DDR		DDRE

#define	DIO2_PIN		PINE4
#define	DIO2_RPORT	PINE
#define	DIO2_WPORT	PORTE
#define	DIO2_DDR		DDRE

#define	DIO3_PIN		PINE5
#define	DIO3_RPORT	PINE
#define	DIO3_WPORT	PORTE
#define	DIO3_DDR		DDRE

#define	DIO4_PIN		PING5
#define	DIO4_RPORT	PING
#define	DIO4_WPORT	PORTG
#define	DIO4_DDR		DDRG

#define	DIO5_PIN		PINE3
#define	DIO5_RPORT	PINE
#define	DIO5_WPORT	PORTE
#define	DIO5_DDR		DDRE

#define	DIO6_PIN		PINH3
#define	DIO6_RPORT	PINH
#define	DIO6_WPORT	PORTH
#define	DIO6_DDR		DDRH

#define	DIO7_PIN		PINH4
#define	DIO7_RPORT	PINH
#define	DIO7_WPORT	PORTH
#define	DIO7_DDR		DDRH

#define	DIO8_PIN		PINH5
#define	DIO8_RPORT	PINH
#define	DIO8_WPORT	PORTH
#define	DIO8_DDR		DDRH

#define	DIO9_PIN		PINH6
#define	DIO9_RPORT	PINH
#define	DIO9_WPORT	PORTH
#define	DIO9_DDR		DDRH

#define	DIO10_PIN		PINB4
#define	DIO10_RPORT	PINB
#define	DIO10_WPORT	PORTB
#define	DIO10_DDR		DDRB

#define	DIO11_PIN		PINB5
#define	DIO11_RPORT	PINB
#define	DIO11_WPORT	PORTB
#define	DIO11_DDR		DDRB

#define	DIO12_PIN		PINB6
#define	DIO12_RPORT	PINB
#define	DIO12_WPORT	PORTB
#define	DIO12_DDR		DDRB

#define	DIO13_PIN		PINB7
#define	DIO13_RPORT	PINB
#define	DIO13_WPORT	PORTB
#define	DIO13_DDR		DDRB

#define	DIO14_PIN		PINJ1
#define	DIO14_RPORT	PINJ
#define	DIO14_WPORT	PORTJ
#define	DIO14_DDR		DDRJ

#define	DIO15_PIN		PINJ0
#define	DIO15_RPORT	PINJ
#define	DIO15_WPORT	PORTJ
#define	DIO15_DDR		DDRJ

#define	DIO16_PIN		PINH1
#define	DIO16_RPORT	PINH
#define	DIO16_WPORT	PORTH
#define	DIO16_DDR		DDRH

#define	DIO17_PIN		PINH0
#define	DIO17_RPORT	PINH
#define	DIO17_WPORT	PORTH
#define	DIO17_DDR		DDRH

#define	DIO18_PIN		PIND3
#define	DIO18_RPORT	PIND
#define	DIO18_WPORT	PORTD
#define	DIO18_DDR		DDRD

#define	DIO19_PIN		PIND2
#define	DIO19_RPORT	PIND
#define	DIO19_WPORT	PORTD
#define	DIO19_DDR		DDRD

#define	DIO20_PIN		PIND1
#define	DIO20_RPORT	PIND
#define	DIO20_WPORT	PORTD
#define	DIO20_DDR		DDRD

#define	DIO21_PIN		PIND0
#define	DIO21_RPORT	PIND
#define	DIO21_WPORT	PORTD
#define	DIO21_DDR		DDRD

#define	DIO22_PIN		PINA0
#define	DIO22_RPORT	PINA
#define	DIO22_WPORT	PORTA
#define	DIO22_DDR		DDRA

#define	DIO23_PIN		PINA1
#define	DIO23_RPORT	PINA
#define	DIO23_WPORT	PORTA
#define	DIO23_DDR		DDRA

#define	DIO24_PIN		PINA2
#define	DIO24_RPORT	PINA
#define	DIO24_WPORT	PORTA
#define	DIO24_DDR		DDRA

#define	DIO25_PIN		PINA3
#define	DIO25_RPORT	PINA
#define	DIO25_WPORT	PORTA
#define	DIO25_DDR		DDRA

#define	DIO26_PIN		PINA4
#define	DIO26_RPORT	PINA
#define	DIO26_WPORT	PORTA
#define	DIO26_DDR		DDRA

#define	DIO27_PIN		PINA5
#define	DIO27_RPORT	PINA
#define	DIO27_WPORT	PORTA
#define	DIO27_DDR		DDRA

#define	DIO28_PIN		PINA6
#define	DIO28_RPORT	PINA
#define	DIO28_WPORT	PORTA
#define	DIO28_DDR		DDRA

#define	DIO29_PIN		PINA7
#define	DIO29_RPORT	PINA
#define	DIO29_WPORT	PORTA
#define	DIO29_DDR		DDRA

#define	DIO30_PIN		PINC7
#define	DIO30_RPORT	PINC
#define	DIO30_WPORT	PORTC
#define	DIO30_DDR		DDRC

#define	DIO31_PIN		PINC6
#define	DIO31_RPORT	PINC
#define	DIO31_WPORT	PORTC
#define	DIO31_DDR		DDRC

#define	DIO32_PIN		PINC5
#define	DIO32_RPORT	PINC
#define	DIO32_WPORT	PORTC
#define	DIO32_DDR		DDRC

#define	DIO33_PIN		PINC4
#define	DIO33_RPORT	PINC
#define	DIO33_WPORT	PORTC
#define	DIO33_DDR		DDRC

#define	DIO34_PIN		PINC3
#define	DIO34_RPORT	PINC
#define	DIO34_WPORT	PORTC
#define	DIO34_DDR		DDRC

#define	DIO35_PIN		PINC2
#define	DIO35_RPORT	PINC
#define	DIO35_WPORT	PORTC
#define	DIO35_DDR		DDRC

#define	DIO36_PIN		PINC1
#define	DIO36_RPORT	PINC
#define	DIO36_WPORT	PORTC
#define	DIO36_DDR		DDRC

#define	DIO37_PIN		PINC0
#define	DIO37_RPORT	PINC
#define	DIO37_WPORT	PORTC
#define	DIO37_DDR		DDRC

#define	DIO38_PIN		PIND7
#define	DIO38_RPORT	PIND
#define	DIO38_WPORT	PORTD
#define	DIO38_DDR		DDRD

#define	DIO39_PIN		PING2
#define	DIO39_RPORT	PING
#define	DIO39_WPORT	PORTG
#define	DIO39_DDR		DDRG

#define	DIO40_PIN		PING1
#define	DIO40_RPORT	PING
#define	DIO40_WPORT	PORTG
#define	DIO40_DDR		DDRG

#define	DIO41_PIN		PING0
#define	DIO41_RPORT	PING
#define	DIO41_WPORT	PORTG
#define	DIO41_DDR		DDRG

#define	DIO42_PIN		PINL7
#define	DIO42_RPORT	PINL
#define	DIO42_WPORT	PORTL
#define	DIO42_DDR		DDRL

#define	DIO43_PIN		PINL6
#define	DIO43_RPORT	PINL
#define	DIO43_WPORT	PORTL
#define	DIO43_DDR		DDRL

#define	DIO44_PIN		PINL5
#define	DIO44_RPORT	PINL
#define	DIO44_WPORT	PORTL
#define	DIO44_DDR		DDRL

#define	DIO45_PIN		PINL4
#define	DIO45_RPORT	PINL
#define	DIO45_WPORT	PORTL
#define	DIO45_DDR		DDRL

#define	DIO46_PIN		PINL3
#define	DIO46_RPORT	PINL
#define	DIO46_WPORT	PORTL
#define	DIO46_DDR		DDRL

#define	DIO47_PIN		PINL2
#define	DIO47_RPORT	PINL
#define	DIO47_WPORT	PORTL
#define	DIO47_DDR		DDRL

#define	DIO48_PIN		PINL1
#define	DIO48_RPORT	PINL
#define	DIO48_WPORT	PORTL
#define	DIO48_DDR		DDRL

#define	DIO49_PIN		PINL0
#define	DIO49_RPORT	PINL
#define	DIO49_WPORT	PORTL
#define	DIO49_DDR		DDRL

#define	DIO50_PIN		PINB3
#define	DIO50_RPORT	PINB
#define	DIO50_WPORT	PORTB
#define	DIO50_DDR		DDRB

#define	DIO51_PIN		PINB2
#define	DIO51_RPORT	PINB
#define	DIO51_WPORT	PORTB
#define	DIO51_DDR		DDRB

#define	DIO52_PIN		PINB1
#define	DIO52_RPORT	PINB
#define	DIO52_WPORT	PORTB
#define	DIO52_DDR		DDRB

#define	DIO53_PIN		PINB0
#define	DIO53_RPORT	PINB
#define	DIO53_WPORT	PORTB
#define	DIO53_DDR		DDRB

#define DIO54_PIN		PINF0
#define DIO54_RPORT	PINF
#define DIO54_WPORT	PORTF
#define DIO54_DDR		DDRF

#define DIO55_PIN		PINF1
#define DIO55_RPORT	PINF
#define DIO55_WPORT	PORTF
#define DIO55_DDR		DDRF

#define DIO56_PIN		PINF2
#define DIO56_RPORT	PINF
#define DIO56_WPORT	PORTF
#define DIO56_DDR		DDRF

#define DIO57_PIN		PINF3
#define DIO57_RPORT	PINF
#define DIO57_WPORT	PORTF
#define DIO57_DDR		DDRF

#define DIO58_PIN		PINF4
#define DIO58_RPORT	PINF
#define DIO58_WPORT	PORTF
#define DIO58_DDR		DDRF

#define DIO59_PIN		PINF5
#define DIO59_RPORT	PINF
#define DIO59_WPORT	PORTF
#define DIO59_DDR		DDRF

#define DIO60_PIN		PINF6
#define DIO60_RPORT	PINF
#define DIO60_WPORT	PORTF
#define DIO60_DDR		DDRF

#define DIO61_PIN		PINF7
#define DIO61_RPORT	PINF
#define DIO61_WPORT	PORTF
#define DIO61_DDR		DDRF

#define DIO62_PIN		PINK0
#define DIO62_RPORT	PINK
#define DIO62_WPORT	PORTK
#define DIO62_DDR		DDRK

#define DIO63_PIN		PINK1
#define DIO63_RPORT	PINK
#define DIO63_WPORT	PORTK
#define DIO63_DDR		DDRK

#define DIO64_PIN		PINK2
#define DIO64_RPORT	PINK
#define DIO64_WPORT	PORTK
#define DIO64_DDR		DDRK

#define DIO65_PIN		PINK3
#define DIO65_RPORT	PINK
#define DIO65_WPORT	PORTK
#define DIO65_DDR		DDRK

#define DIO66_PIN		PINK4
#define DIO66_RPORT	PINK
#define DIO66_WPORT	PORTK
#define DIO66_DDR		DDRK

#define DIO67_PIN		PINK5
#define DIO67_RPORT	PINK
#define DIO67_WPORT	PORTK
#define DIO67_DDR		DDRK

#define DIO68_PIN		PINK6
#define DIO68_RPORT	PINK
#define DIO68_WPORT	PORTK
#define DIO68_DDR		DDRK

#define DIO69_PIN		PINK7
#define DIO69_RPORT	PINK
#define DIO69_WPORT	PORTK
#define DIO69_DDR		DDRK


/** Set of defines that allow the AVR pin names to be used in the WRITE/READ/SET_OUTPUT/SET_INPUT macros */
#define DIOA0_PIN    PINA0
#define DIOA0_RPORT  PINA
#define DIOA0_WPORT  PORTA
#define DIOA0_DDR    DDRA

#define DIOA1_PIN    PINA1
#define DIOA1_RPORT  PINA
#define DIOA1_WPORT  PORTA
#define DIOA1_DDR    DDRA

#define DIOA2_PIN    PINA2
#define DIOA2_RPORT  PINA
#define DIOA2_WPORT  PORTA
#define DIOA2_DDR    DDRA

#define DIOA3_PIN    PINA3
#define DIOA3_RPORT  PINA
#define DIOA3_WPORT  PORTA
#define DIOA3_DDR    DDRA

#define DIOA4_PIN    PINA4
#define DIOA4_RPORT  PINA
#define DIOA4_WPORT  PORTA
#define DIOA4_DDR    DDRA

#define DIOA5_PIN    PINA5
#define DIOA5_RPORT  PINA
#define DIOA5_WPORT  PORTA
#define DIOA5_DDR    DDRA

#define DIOA6_PIN    PINA6
#define DIOA6_RPORT  PINA
#define DIOA6_WPORT  PORTA
#define DIOA6_DDR    DDRA

#define DIOA7_PIN    PINA7
#define DIOA7_RPORT  PINA
#define DIOA7_WPORT  PORTA
#define DIOA7_DDR    DDRA

#define DIOB0_PIN    PINB0
#define DIOB0_RPORT  PINB
#define DIOB0_WPORT  PORTB
#define DIOB0_DDR    DDRB

#define DIOB1_PIN    PINB1
#define DIOB1_RPORT  PINB
#define DIOB1_WPORT  PORTB
#define DIOB1_DDR    DDRB

#define DIOB2_PIN    PINB2
#define DIOB2_RPORT  PINB
#define DIOB2_WPORT  PORTB
#define DIOB2_DDR    DDRB

#define DIOB3_PIN    PINB3
#define DIOB3_RPORT  PINB
#define DIOB3_WPORT  PORTB
#define DIOB3_DDR    DDRB

#define DIOB4_PIN    PINB4
#define DIOB4_RPORT  PINB
#define DIOB4_WPORT  PORTB
#define DIOB4_DDR    DDRB

#define DIOB5_PIN    PINB5
#define DIOB5_RPORT  PINB
#define DIOB5_WPORT  PORTB
#define DIOB5_DDR    DDRB

#define DIOB6_PIN    PINB6
#define DIOB6_RPORT  PINB
#define DIOB6_WPORT  PORTB
#define DIOB6_DDR    DDRB

#define DIOB7_PIN    PINB7
#define DIOB7_RPORT  PINB
#define DIOB7_WPORT  PORTB
#define DIOB7_DDR    DDRB

#define DIOC0_PIN    PINC0
#define DIOC0_RPORT  PINC
#define DIOC0_WPORT  PORTC
#define DIOC0_DDR    DDRC

#define DIOC1_PIN    PINC1
#define DIOC1_RPORT  PINC
#define DIOC1_WPORT  PORTC
#define DIOC1_DDR    DDRC

#define DIOC2_PIN    PINC2
#define DIOC2_RPORT  PINC
#define DIOC2_WPORT  PORTC
#define DIOC2_DDR    DDRC

#define DIOC3_PIN    PINC3
#define DIOC3_RPORT  PINC
#define DIOC3_WPORT  PORTC
#define DIOC3_DDR    DDRC

#define DIOC4_PIN    PINC4
#define DIOC4_RPORT  PINC
#define DIOC4_WPORT  PORTC
#define DIOC4_DDR    DDRC

#define DIOC5_PIN    PINC5
#define DIOC5_RPORT  PINC
#define DIOC5_WPORT  PORTC
#define DIOC5_DDR    DDRC

#define DIOC6_PIN    PINC6
#define DIOC6_RPORT  PINC
#define DIOC6_WPORT  PORTC
#define DIOC6_DDR    DDRC

#define DIOC7_PIN    PINC7
#define DIOC7_RPORT  PINC
#define DIOC7_WPORT  PORTC
#define DIOC7_DDR    DDRC

#define DIOD0_PIN    PIND0
#define DIOD0_RPORT  PIND
#define DIOD0_WPORT  PORTD
#define DIOD0_DDR    DDRD

#define DIOD1_PIN    PIND1
#define DIOD1_RPORT  PIND
#define DIOD1_WPORT  PORTD
#define DIOD1_DDR    DDRD

#define DIOD2_PIN    PIND2
#define DIOD2_RPORT  PIND
#define DIOD2_WPORT  PORTD
#define DIOD2_DDR    DDRD

#define DIOD3_PIN    PIND3
#define DIOD3_RPORT  PIND
#define DIOD3_WPORT  PORTD
#define DIOD3_DDR    DDRD

#define DIOD4_PIN    PIND4
#define DIOD4_RPORT  PIND
#define DIOD4_WPORT  PORTD
#define DIOD4_DDR    DDRD

#define DIOD5_PIN    PIND5
#define DIOD5_RPORT  PIND
#define DIOD5_WPORT  PORTD
#define DIOD5_DDR    DDRD

#define DIOD6_PIN    PIND6
#define DIOD6_RPORT  PIND
#define DIOD6_WPORT  PORTD
#define DIOD6_DDR    DDRD

#define DIOD7_PIN    PIND7
#define DIOD7_RPORT  PIND
#define DIOD7_WPORT  PORTD
#define DIOD7_DDR    DDRD

#define DIOE0_PIN    PINE0
#define DIOE0_RPORT  PINE
#define DIOE0_WPORT  PORTE
#define DIOE0_DDR    DDRE

#define DIOE1_PIN    PINE1
#define DIOE1_RPORT  PINE
#define DIOE1_WPORT  PORTE
#define DIOE1_DDR    DDRE

#define DIOE2_PIN    PINE2
#define DIOE2_RPORT  PINE
#define DIOE2_WPORT  PORTE
#define DIOE2_DDR    DDRE

#define DIOE3_PIN    PINE3
#define DIOE3_RPORT  PINE
#define DIOE3_WPORT  PORTE
#define DIOE3_DDR    DDRE

#define DIOE4_PIN    PINE4
#define DIOE4_RPORT  PINE
#define DIOE4_WPORT  PORTE
#define DIOE4_DDR    DDRE

#define DIOE5_PIN    PINE5
#define DIOE5_RPORT  PINE
#define DIOE5_WPORT  PORTE
#define DIOE5_DDR    DDRE

#define DIOE6_PIN    PINE6
#define DIOE6_RPORT  PINE
#define DIOE6_WPORT  PORTE
#define DIOE6_DDR    DDRE

#define DIOE7_PIN    PINE7
#define DIOE7_RPORT  PINE
#define DIOE7_WPORT  PORTE
#define DIOE7_DDR    DDRE

#define DIOF0_PIN    PINF0
#define DIOF0_RPORT  PINF
#define DIOF0_WPORT  PORTF
#define DIOF0_DDR    DDRF

#define DIOF1_PIN    PINF1
#define DIOF1_RPORT  PINF
#define DIOF1_WPORT  PORTF
#define DIOF1_DDR    DDRF

#define DIOF2_PIN    PINF2
#define DIOF2_RPORT  PINF
#define DIOF2_WPORT  PORTF
#define DIOF2_DDR    DDRF

#define DIOF3_PIN    PINF3
#define DIOF3_RPORT  PINF
#define DIOF3_WPORT  PORTF
#define DIOF3_DDR    DDRF

#define DIOF4_PIN    PINF4
#define DIOF4_RPORT  PINF
#define DIOF4_WPORT  PORTF
#define DIOF4_DDR    DDRF

#define DIOF5_PIN    PINF5
#define DIOF5_RPORT  PINF
#define DIOF5_WPORT  PORTF
#define DIOF5_DDR    DDRF

#define DIOF6_PIN    PINF6
#define DIOF6_RPORT  PINF
#define DIOF6_WPORT  PORTF
#define DIOF6_DDR    DDRF

#define DIOF7_PIN    PINF7
#define DIOF7_RPORT  PINF
#define DIOF7_WPORT  PORTF
#define DIOF7_DDR    DDRF

#define DIOG0_PIN    PING0
#define DIOG0_RPORT  PING
#define DIOG0_WPORT  PORTG
#define DIOG0_DDR    DDRG

#define DIOG1_PIN    PING1
#define DIOG1_RPORT  PING
#define DIOG1_WPORT  PORTG
#define DIOG1_DDR    DDRG

#define DIOG2_PIN    PING2
#define DIOG2_RPORT  PING
#define DIOG2_WPORT  PORTG
#define DIOG2_DDR    DDRG

#define DIOG3_PIN    PING3
#define DIOG3_RPORT  PING
#define DIOG3_WPORT  PORTG
#define DIOG3_DDR    DDRG

#define DIOG4_PIN    PING4
#define DIOG4_RPORT  PING
#define DIOG4_WPORT  PORTG
#define DIOG4_DDR    DDRG

#define DIOG5_PIN    PING5
#define DIOG5_RPORT  PING
#define DIOG5_WPORT  PORTG
#define DIOG5_DDR    DDRG

#define DIOG6_PIN    PING6
#define DIOG6_RPORT  PING
#define DIOG6_WPORT  PORTG
#define DIOG6_DDR    DDRG

#define DIOG7_PIN    PING7
#define DIOG7_RPORT  PING
#define DIOG7_WPORT  PORTG
#define DIOG7_DDR    DDRG

#define DIOH0_PIN    PINH0
#define DIOH0_RPORT  PINH
#define DIOH0_WPORT  PORTH
#define DIOH0_DDR    DDRH

#define DIOH1_PIN    PINH1
#define DIOH1_RPORT  PINH
#define DIOH1_WPORT  PORTH
#define DIOH1_DDR    DDRH

#define DIOH2_PIN    PINH2
#define DIOH2_RPORT  PINH
#define DIOH2_WPORT  PORTH
#define DIOH2_DDR    DDRH

#define DIOH3_PIN    PINH3
#define DIOH3_RPORT  PINH
#define DIOH3_WPORT  PORTH
#define DIOH3_DDR    DDRH

#define DIOH4_PIN    PINH4
#define DIOH4_RPORT  PINH
#define DIOH4_WPORT  PORTH
#define DIOH4_DDR    DDRH

#define DIOH5_PIN    PINH5
#define DIOH5_RPORT  PINH
#define DIOH5_WPORT  PORTH
#define DIOH5_DDR    DDRH

#define DIOH6_PIN    PINH6
#define DIOH6_RPORT  PINH
#define DIOH6_WPORT  PORTH
#define DIOH6_DDR    DDRH

#define DIOH7_PIN    PINH7
#define DIOH7_RPORT  PINH
#define DIOH7_WPORT  PORTH
#define DIOH7_DDR    DDRH

#define DIOJ0_PIN    PINJ0
#define DIOJ0_RPORT  PINJ
#define DIOJ0_WPORT  PORTJ
#define DIOJ0_DDR    DDRJ

#define DIOJ1_PIN    PINJ1
#define DIOJ1_RPORT  PINJ
#define DIOJ1_WPORT  PORTJ
#define DIOJ1_DDR    DDRJ

#define DIOJ2_PIN    PINJ2
#define DIOJ2_RPORT  PINJ
#define DIOJ2_WPORT  PORTJ
#define DIOJ2_DDR    DDRJ

#define DIOJ3_PIN    PINJ3
#define DIOJ3_RPORT  PINJ
#define DIOJ3_WPORT  PORTJ
#define DIOJ3_DDR    DDRJ

#define DIOJ4_PIN    PINJ4
#define DIOJ4_RPORT  PINJ
#define DIOJ4_WPORT  PORTJ
#define DIOJ4_DDR    DDRJ

#define DIOJ5_PIN    PINJ5
#define DIOJ5_RPORT  PINJ
#define DIOJ5_WPORT  PORTJ
#define DIOJ5_DDR    DDRJ

#define DIOJ6_PIN    PINJ6
#define DIOJ6_RPORT  PINJ
#define DIOJ6_WPORT  PORTJ
#define DIOJ6_DDR    DDRJ

#define DIOJ7_PIN    PINJ7
#define DIOJ7_RPORT  PINJ
#define DIOJ7_WPORT  PORTJ
#define DIOJ7_DDR    DDRJ

#define DIOK0_PIN    PINK0
#define DIOK0_RPORT  PINK
#define DIOK0_WPORT  PORTK
#define DIOK0_DDR    DDRK

#define DIOK1_PIN    PINK1
#define DIOK1_RPORT  PINK
#define DIOK1_WPORT  PORTK
#define DIOK1_DDR    DDRK

#define DIOK2_PIN    PINK2
#define DIOK2_RPORT  PINK
#define DIOK2_WPORT  PORTK
#define DIOK2_DDR    DDRK

#define DIOK3_PIN    PINK3
#define DIOK3_RPORT  PINK
#define DIOK3_WPORT  PORTK
#define DIOK3_DDR    DDRK

#define DIOK4_PIN    PINK4
#define DIOK4_RPORT  PINK
#define DIOK4_WPORT  PORTK
#define DIOK4_DDR    DDRK

#define DIOK5_PIN    PINK5
#define DIOK5_RPORT  PINK
#define DIOK5_WPORT  PORTK
#define DIOK5_DDR    DDRK

#define DIOK6_PIN    PINK6
#define DIOK6_RPORT  PINK
#define DIOK6_WPORT  PORTK
#define DIOK6_DDR    DDRK

#define DIOK7_PIN    PINK7
#define DIOK7_RPORT  PINK
#define DIOK7_WPORT  PORTK
#define DIOK7_DDR    DDRK

#define DIOL0_PIN    PINL0
#define DIOL0_RPORT  PINL
#define DIOL0_WPORT  PORTL
#define DIOL0_DDR    DDRL

#define DIOL1_PIN    PINL1
#define DIOL1_RPORT  PINL
#define DIOL1_WPORT  PORTL
#define DIOL1_DDR    DDRL

#define DIOL2_PIN    PINL2
#define DIOL2_RPORT  PINL
#define DIOL2_WPORT  PORTL
#define DIOL2_DDR    DDRL

#define DIOL3_PIN    PINL3
#define DIOL3_RPORT  PINL
#define DIOL3_WPORT  PORTL
#define DIOL3_DDR    DDRL

#define DIOL4_PIN    PINL4
#define DIOL4_RPORT  PINL
#define DIOL4_WPORT  PORTL
#define DIOL4_DDR    DDRL

#define DIOL5_PIN    PINL5
#define DIOL5_RPORT  PINL
#define DIOL5_WPORT  PORTL
#define DIOL5_DDR    DDRL

#define DIOL6_PIN    PINL6
#define DIOL6_RPORT  PINL
#define DIOL6_WPORT  PORTL
#define DIOL6_DDR    DDRL

#define DIOL7_PIN    PINL7
#define DIOL7_RPORT  PINL
#define DIOL7_WPORT  PORTL
#define DIOL7_DDR    DDRL

#endif


#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please submit a pull request
#endif

#endif /* _FASTIO_ARDUINO_H */

