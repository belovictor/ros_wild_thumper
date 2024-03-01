#!/usr/bin/ruby -I ~/ruby_test/.bundle/ruby/2.3.0/gems/i2c-0.4.2/lib

require 'rubygems'
require 'bundler/setup'
require "i2c"

i2c = I2C.create("/dev/i2c-2") 

a = i2c.read(0x50>>1, 1, 0xa3)
puts "0x%x" % a.unpack("C")
