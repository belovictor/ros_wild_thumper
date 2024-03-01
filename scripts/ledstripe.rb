#!/usr/bin/env ruby

require "spi"
require "ros"
require 'wild_thumper/LedStripe'

class LPD8806
	def initialize(device, num_leds)
		@spi = SPI.new(device: device)
		@spi.driver.mode=[0b00]
		@spi.speed=2e6
		@num_leds = num_leds
		self.latch()
		@l = [[0, 0, 0]] * num_leds
		self.update()
	end
	
	def set(i, red=0, green=0, blue=0)
		if red > 127 or green > 127 or blue > 127 or red < 0 or green < 0 or blue < 0
			raise "Bad RGB Value"
		end
		@l[i] = [red, green, blue]
	end

	def latch
		@spi.xfer(txdata: (0...(@num_leds+31)/32).map { |i| 0 })
	end
	
	def update
		l = []
		(0...@num_leds).each do |i|
			red, green, blue = @l[i]
			l.push(0x80 | green)
			l.push(0x80 | red)
			l.push(0x80 | blue)
		end
		@spi.xfer(txdata: l)
		self.latch()
	end
end

if __FILE__ == $0
	def test
		puts "Starting.."
		(0...16).each do |i|
			stripe.set(i, 10, 0, 0)
			stripe.update()
			sleep(0.1)
		end
		(0...16).each do |i|
			stripe.set(i, 0, 10, 0)
			stripe.update()
			sleep(0.1)
		end
		(0...16).each do |i|
			stripe.set(i, 0, 0, 10)
			stripe.update()
			sleep(0.1)
		end
		(0...16).each do |i|
			stripe.set(i, 0, 0, 0)
		end
		stripe.update()
	end

	stripe = LPD8806.new("/dev/spidev1.0", 16)
	node = ROS::Node.new('wild_thumper/led_stripe')
	node.subscribe('led_stripe', Wild_thumper::LedStripe) do |msg|
		msg.leds.each do |led|
			stripe.set(led.num, red=led.red, green=led.green, blue=led.blue)
			stripe.update()
		end
	end

	while node.ok?
		node.spin
	end
end
