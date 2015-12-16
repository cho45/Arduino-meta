/*
  Arduino-meta.hpp - A metaprogrammed Arduino functions.

  Copyright (c) 2015 cho45 - http://www.lowreal.net/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/
#ifndef Arduino_meta_hpp
#define Arduino_meta_hpp

#include <Arduino.h>

// defined in Arduino.h but this is not seen in this context.
static constexpr uint8_t PB = 2;
static constexpr uint8_t PC = 3;
static constexpr uint8_t PD = 4;

// template version of Arduino builtin functions
// standard pins
static constexpr uint16_t port_to_mode[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

static constexpr uint16_t port_to_output[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

static constexpr uint16_t port_to_input[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

static constexpr uint8_t digital_pin_to_port[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
};

static constexpr uint8_t digital_pin_to_bit_mask[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};

static constexpr uint8_t digital_pin_to_timer[] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	// on the ATmega168, digital pin 3 has hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
#else
	TIMER2B,
#endif
	NOT_ON_TIMER,
	// on the ATmega168, digital pins 5 and 6 have hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
	NOT_ON_TIMER,
#else
	TIMER0B,
	TIMER0A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
#if defined(__AVR_ATmega8__)
	TIMER2,
#else
	TIMER2A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 14 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

template<uint8_t port>
static constexpr volatile uint8_t* portModeRegisterX() {
	static_assert(port < sizeof(port_to_mode), "invalid port number");
	static_assert(port_to_mode[port] != NOT_A_PORT, "invalid port number");
	return (volatile uint8_t*)port_to_mode[port];
}

template<uint8_t port>
static constexpr volatile uint8_t* portOutputRegisterX() {
	static_assert(port < sizeof(port_to_output), "invalid port number");
	static_assert(port_to_output[port] != NOT_A_PORT, "invalid port number");
	return (volatile uint8_t*)port_to_output[port];
}

template<uint8_t port>
static constexpr volatile uint8_t* portInputRegisterX() {
	static_assert(port < sizeof(port_to_input), "invalid port number");
	static_assert(port_to_input[port] != NOT_A_PORT, "invalid port number");
	return (volatile uint8_t*)port_to_input[port];
}

template <uint16_t pin>
static constexpr uint8_t digitalPinToBitMaskX() {
	static_assert(pin < sizeof(digital_pin_to_bit_mask), "invalid pin number");
	return digital_pin_to_bit_mask[pin];
};


template <uint16_t pin>
static constexpr uint8_t digitalPinToPortX() {
	static_assert(pin < sizeof(digital_pin_to_port), "invalid pin number");
	return digital_pin_to_port[pin];
};

template <uint16_t pin>
static constexpr uint8_t digitalPinToTimerX() {
	static_assert(pin < sizeof(digital_pin_to_timer), "invalid pin number");
	return digital_pin_to_timer[pin];
};

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

template <uint8_t timer>
static inline void turnOffPWM() {
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif

		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif

		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif

		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif

		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

// multiple bit set function with cheking port consistency

/**
 * port = target port
 * mask = bit mask which should be updated
 * vals = actual bit values for updating
 */
template<uint8_t port, uint8_t mask, uint8_t vals>
static inline void digitalWriteMulti_() {
	volatile uint8_t *out = portOutputRegisterX<port>();
	uint8_t oldSREG = SREG;
	cli();
	*out = (*out & ~mask) | vals;
	SREG = oldSREG;
}

template<uint8_t port, uint8_t mask, uint8_t vals, uint16_t pin, uint8_t val, uint16_t... Rest>
static inline void digitalWriteMulti_() {
	static_assert(digitalPinToPortX<pin>() == port, "all port must be same");
	constexpr uint8_t bit  = digitalPinToBitMaskX<pin>();
	static_assert( (bit & mask) == 0, "already specified pin");
	constexpr uint8_t timer = digitalPinToTimerX<pin>();
	if (timer != NOT_ON_TIMER) turnOffPWM<timer>();
	digitalWriteMulti_<
		port,
		mask | bit,
		vals | (val ? bit : 0),
		Rest...
	>();
}

template<uint16_t pin, uint8_t val, uint16_t... Rest>
static inline void digitalWriteMulti() {
	constexpr uint8_t port = digitalPinToPortX<pin>();
	return digitalWriteMulti_<
		port,
		0,
		0,

		pin,
		val,
		Rest...
	>();
}


/**
 * port = target port
 * modeMask = bit mask which should be updated
 * modeVals = actual bit values for updating
 * outMask = bit mask which should be updated (only change with entering INPUT)
 * outVals = actual bit values for updating
 */
template<uint8_t port, uint8_t modeMask, uint8_t modeVals, uint8_t outMask, uint8_t outVals>
static inline void pinModeMulti_() {
	volatile uint8_t *ddr = portModeRegisterX<port>();
	volatile uint8_t *out = portOutputRegisterX<port>();
	*ddr = (*ddr & ~modeMask) | modeVals;
	if (outMask) {
		*out = (*out & ~outMask) | outVals;
	}
}

template<uint8_t port, uint8_t modeMask, uint8_t modeVals, uint8_t outMask, uint8_t outVals, uint16_t pin, uint8_t mode, uint16_t... Rest>
static inline void pinModeMulti_() {
	static_assert(digitalPinToPortX<pin>() == port, "all port must be same");
	constexpr uint8_t bit = digitalPinToBitMaskX<pin>();
	static_assert( (bit & modeMask) == 0, "already specified pin");
	pinModeMulti_<
		port,
		modeMask | bit,
		modeVals | (mode == OUTPUT ? bit : 0),
		outMask | (mode == OUTPUT ? 0 : bit),
		outVals | (mode == INPUT_PULLUP ? bit : 0),
		Rest...
	>();
}

template<uint16_t pin, uint8_t mode, uint16_t... Rest>
static inline void pinModeMulti() {
	constexpr uint8_t port = digitalPinToPortX<pin>();
	return pinModeMulti_<
		port,
		0,
		0,
		0,
		0,

		pin,
		mode,
		Rest...
	>();
}

template<uint16_t pin>
static inline int digitalReadX() {
	constexpr uint8_t port = digitalPinToPortX<pin>();
	constexpr uint8_t timer = digitalPinToTimerX<pin>();
	constexpr uint8_t bit = digitalPinToBitMaskX<pin>();
	constexpr volatile uint8_t *in = portInputRegisterX<port>();
	if (timer != NOT_ON_TIMER) turnOffPWM<timer>();
	return (*in & bit) ? HIGH : LOW;
}

// hide template magic from interface
// and use static function by default.
// redefine original dynamic functions to *Dynamic().
static constexpr void (*digitalWriteDynamic)(uint8_t, uint8_t)  = *digitalWrite;
#define digitalWrite(...) digitalWriteMulti<__VA_ARGS__>()
static constexpr void (*pinModeDynamic)(uint8_t, uint8_t)  = *pinMode;
#define pinMode(...) pinModeMulti<__VA_ARGS__>()
static constexpr int (*digitalReadDynamic)(uint8_t)  = *digitalRead;
#define digitalRead(pin) digitalReadX<pin>()


#endif
