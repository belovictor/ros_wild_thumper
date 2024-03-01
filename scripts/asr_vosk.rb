#!/usr/bin/ruby
# export AUDIODEV=plughw:CARD=ArrayUAC10,0
# rec -q -t alsa -c 1 -b 16 -r 16000 -t wav - silence -l 1 0.1 0.3% -1 2.0 0.3% | ./asr_vosk.rb -

require 'logger'
require 'websocket-eventmachine-client'
require 'json'
require 'ros'
require 'std_msgs/String'

KEYWORDS = ["wild thumper"]
CONFIG = {
	"config": {
		"phrase_list": ["angle", "backward", "by", "centimeter", "compass", "current", "decrease", "default", "degree", "down", "eight", "eighteen", "eighty", "eleven", "fifteen", "fifty", "five", "forty", "forward", "four", "fourteen", "get", "go", "hundred", "increase", "left", "light", "lights", "meter", "mic", "minus", "motion", "mute", "nine", "nineteen", "ninety", "off", "on", "one", "position", "pressure", "right", "secure", "set", "seven", "seventeen", "seventy", "silence", "six", "sixteen", "sixty", "speed", "stop", "temp", "temperature", "ten", "thirteen", "thirty", "three", "to", "turn", "twelve", "twenty", "two", "up", "velocity", "voltage", "volume", "wild thumper", "zero"],
		"sample_rate": 16000.0
	}
}

class Speak
	def initialize(node)
		@logger = Logger.new(STDOUT)
		@commands_enabled = false
		@publisher = node.advertise('asr_result', Std_msgs::String)

		# Websocket handling
		EM.run do
			Signal.trap("INT") { send_eof }
			@ws = WebSocket::EventMachine::Client.connect(:uri => 'ws://192.168.36.4:2700')

			def send_eof
				@ws.send '{"eof" : 1}'
			end

			# Loop over all input data
			def run
				while true do
					data = ARGF.read(16000)
					if data
						@ws.send data, :type => :binary
					else
						send_eof
						break
					end
				end
			end

			@ws.onopen do
				@logger.info "Running.."
				@ws.send CONFIG.to_json

				Thread.new {
					run
				}
			end

			@ws.onmessage do |msg, type|
				d = JSON.parse(msg)
				handle_result(d)
			end

			@ws.onclose do |code, reason|
				puts "Disconnected with status code: #{code}"
				exit
			end
		end
	end

	def handle_result(msg)
		if msg.has_key? "result"
			msg["result"].each do |result|
				@logger.debug "word=" + result["word"]
			end

			# check for keywords first
			text = msg["text"]
			@logger.debug "text=" + msg["text"]
			KEYWORDS.each do |keyword|
				if text.include? keyword
					keyword_detected(keyword)
					text = text.gsub(keyword, "").strip
				end
			end

			# not a keyword, handle command if enabled
			if @commands_enabled and text.length > 0
				final_result(text)
			end
		end
	end

	# Enables/Disables the speech command
	def enable_commands(bEnable)
		@commands_enabled = bEnable
	end

	# Resulting speech command
	def final_result(hyp)
		@logger.info "final: " + hyp
		enable_commands(false)

		# Publish vosk result as ros message
		msg = Std_msgs::String.new
		msg.data = hyp
		@publisher.publish(msg)
	end

	def keyword_detected(hyp)
		@logger.debug "Got keyword: " + hyp
		enable_commands(true)
	end
end

if __FILE__ == $0
	node = ROS::Node.new('asr_vosk')
	app = Speak.new(node)
	begin
		node.spin
	rescue Interrupt
	ensure
		node.shutdown
	end
end
