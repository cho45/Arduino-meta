#!/usr/bin/env ruby

require "tmpdir"
require "fileutils"
require "pp"

ARDUINO_HARDWARE_ROOT = "#{ENV['ARDUINO_DIR']}/hardware"
META_HPP = "Arduino-meta.hpp"

@command = "#{ARDUINO_HARDWARE_ROOT}/tools/avr/bin/avr-g++ -MMD -c -mmcu=atmega168 -DF_CPU=16000000L -DARDUINO=166 -DARDUINO_ARCH_AVR -D__PROG_TYPES_COMPAT__ -I#{ARDUINO_HARDWARE_ROOT}/arduino/avr/cores/arduino -I#{ARDUINO_HARDWARE_ROOT}/arduino/avr/variants/eightanaloginputs -Wall -ffunction-sections -fdata-sections -Os -fno-exceptions -std=gnu++11"
@objdump = "#{ARDUINO_HARDWARE_ROOT}/tools/avr/bin/avr-objdump"

$tests = 0
at_exit do
	puts "1..#{$tests}"
end

def test(source, with_error: nil)
	$tests += 1
	Dir.mktmpdir do |dir|
		FileUtils.cp(META_HPP, dir)
		Dir.chdir(dir) do
			File.open("#{dir}/test.cpp", "w") do |f|
				f.puts '#include "Arduino-meta.hpp"'
				f.puts 'extern "C" { void expected(void); void result(void); }'
				f.puts source
			end
			cmd = "#{@command} test.cpp -o test.o 2>&1"
			# puts "# #{cmd}"
			result = `#{cmd}`
			if $?.success?
				if with_error
					puts "not ok - #{with_error} expected but compiled successfully"
					return
				end
			else
				if with_error
					if result[with_error]
						puts "ok - #{with_error} occured"
					else
						puts "not ok - #{with_error} expected but not occured"
						puts result.gsub(/^/, "# ")
					end
				else
					puts "not ok # compile error"
					puts result.gsub(/^/, "# ")
				end
				return
			end

			dumped = `#{@objdump} -d test.o`
			sections = dumped.split(/\n\n/)

			expected = sections.find {|s| s.sub!(/^00000000 <expected>:/, '') }.chomp
			result = sections.find {|s| s.sub!(/^00000000 <result>:/, '') }.chomp

			if expected == result
				puts "ok"
			else
				puts "not ok # failed"
				puts "# expected:"
				puts expected.gsub(/^/, "# ")
				puts "# result:"
				puts result.gsub(/^/, "# ")
			end
		end
	end
end

puts "# digitalWrite"

test(<<-'EOS')
	void expected() {
		uint8_t oldSREG = SREG;
		cli();
		PORTD = (PORTD & 0b11111110) | (0b00000001);
		SREG = oldSREG;
	}

	void result() {
		digitalWrite(0, 1);
	}
EOS

test(<<-'EOS')
	void expected() {
		uint8_t oldSREG = SREG;
		cli();
		PORTD = (PORTD & 0b11111000) | (0b00000101);
		SREG = oldSREG;
	}

	void result() {
		digitalWrite(0, HIGH, 1, LOW, 2, HIGH);
	}
EOS

# with pwm timer off
test(<<-'EOS')
	void expected() {
		cbi(TCCR1A, COM1A1);
		cbi(TCCR1A, COM1B1);

		uint8_t oldSREG = SREG;
		cli();
		PORTB = (PORTB & 0b11111001) | (0b00000010);
		SREG = oldSREG;
	}

	void result() {
		digitalWrite(9, HIGH, 10, LOW);
	}
EOS

test(<<-'EOS', with_error: 'all port must be same')
	void setup() {
		digitalWrite(0, HIGH, 9, HIGH);
	}
EOS

test(<<-'EOS', with_error: 'already specified pin')
	void setup() {
		digitalWrite(0, HIGH, 0, LOW);
	}
EOS


puts "# pinMode"
test(<<-'EOS')
	void expected() {
		DDRD = (DDRD & 0b11111110) | (0b00000001);
	}

	void result() {
		pinMode(0, OUTPUT);
	}
EOS

test(<<-'EOS')
	void expected() {
		DDRD = (DDRD & 0b11111100) | (0b00000011);
	}

	void result() {
		pinMode(0, OUTPUT, 1, OUTPUT);
	}
EOS

test(<<-'EOS')
	void expected() {
		DDRD  = (DDRD  & 0b11111100) | (0b00000000);
		PORTD = (PORTD & 0b11111100) | (0b00000010);
	}

	void result() {
		pinMode(0, INPUT, 1, INPUT_PULLUP);
	}
EOS


test(<<-'EOS', with_error: 'all port must be same')
	void setup() {
		pinMode(0, OUTPUT, 9, OUTPUT);
	}
EOS

test(<<-'EOS', with_error: 'already specified pin')
	void setup() {
		pinMode(0, OUTPUT, 0, INPUT);
	}
EOS

puts "# digitalRead"
test(<<-'EOS')
	volatile uint8_t r;
	void expected() {
		r = PIND & 0b00000001 ? HIGH : LOW;
	}

	void result() {
		r = digitalRead(0);
	}
EOS

# with pwm timer off
test(<<-'EOS')
	volatile uint8_t r;
	void expected() {
		cbi(TCCR1A, COM1A1);
		r = PINB & 0b00000010 ? HIGH : LOW;
	}

	void result() {
		r = digitalRead(9);
	}
EOS
