#include <Arduino.h>
#include "Arduino-meta.hpp"

constexpr uint8_t LED_RED = 2;
constexpr uint8_t LED_GREEN = 3;
constexpr uint8_t LED_BLUE = 4;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);

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

	// static
	digitalWrite(LED_BUILTIN, HIGH);
	delay(1000);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);

	// original version of digitalWrite()
	digitalWriteDynamic(LED_BUILTIN, HIGH);
	delay(1000);
	digitalWriteDynamic(LED_BUILTIN, LOW);
	delay(1000);
}

