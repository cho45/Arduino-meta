Arduin-meta.hpp
===============

A metaprogrammed Arduino functions.

Synopsys
========

	#include "Arduino.h"
	#include "Arduino-meta.hpp"

	constexpr uint8_t LED_RED = 0;
	constexpr uint8_t LED_GREEN = 1;
	constexpr uint8_t LED_BLUE = 2;

	void main() {
		// static version of pinMode()
		pinMode(
			LED_RED, OUTPUT,
			LED_GREEN, OUTPUT,
			LED_BLUE, OUTPUT
		);
	}

	void loop() {
		// this is also static
		digitalWrite(LED_RED, HIGH, LED_GREEN, LOW, LED_BLUE, LOW);
		delay(1000);
		digitalWrite(LED_RED, LOW, LED_GREEN, HIGH, LED_BLUE, LOW);
		delay(1000);
		digitalWrite(LED_RED, LOW, LED_GREEN, LOW, LED_BLUE, HIGH);
		delay(1000);

		// original version of digitalWrite()
		digitalWriteDynamic(LED_BUILTIN, HIGH);
		delay(1000);
		digitalWriteDynamic(LED_BUILTIN, LOW);
		delay(1000);
	}

Usage
=====

Download Arduin-meta.hpp and

	#include "Arduino-meta.hpp"

This file uses C++11 features so you must specify "-std=c++11" to compiler by "compiler.cpp.flags=...". See also:

 * http://stackoverflow.com/questions/16224746/how-to-use-c11-to-program-the-arduino

digitalWrite / pinMode / digitalRead is replaced with static (template) version.
You can use original version by digitalWriteDynamic / pinModeDynamic / digitalReadDynamic.

Background
==========

Arduino's digitalWrite/pinMode/digitalRead always lookup tables in program memory to get a port register and a bit mask from a pin number.
In most every case, thease functions arguments are static so looking up tables in runtime is needless.

This header file solve it at compile time with C++ templates.

Compile Time Error
==================

If you specify invalid arguments to functions.
This library report it as compile error. This is useful.


Test
====

	make test


LISENCE
=======

Same as Arduino (LGPL)
