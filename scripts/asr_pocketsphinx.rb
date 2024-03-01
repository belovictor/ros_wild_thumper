#!/usr/bin/ruby

require 'gst'
require 'pry'
require 'logger'
require 'ros'
require 'std_msgs/String'

class Speak
	def initialize(node)
		@logger = Logger.new(STDOUT)
		@publisher = node.advertise('asr_result', Std_msgs::String)
		@pipeline = Gst.parse_launch('alsasrc device="plughw:1,0" ! audio/x-raw,format=S16LE,channels=1,rate=16000 ! cutter leaky=true name=cutter'\
					     ' ! tee name=jsgf ! queue leaky=downstream ! valve name=valve_jsgf drop=true ! pocketsphinx name=asr_jsgf ! fakesink async=false jsgf.'\
					     ' ! pocketsphinx name=asr_kws ! fakesink async=false'\
					    )
		# Ignore everything below the configured volume
		cutter = @pipeline.get_by_name('cutter')
		cutter.set_property('threshold-dB', -20)
		cutter.set_property('pre-length', 100000000) # pocketsphinx needs about 0.1s before start
		cutter.set_property('run-length', 1300000000)

		asr_jsgf = @pipeline.get_by_name('asr_jsgf')
		asr_jsgf.set_property('hmm', 'pocketsphinx/adapt/cmusphinx-en-us-5.2')
		asr_jsgf.set_property('mllr', 'pocketsphinx/adapt/mllr_matrix')
		asr_jsgf.set_property('jsgf', 'data/robot.jsgf')

		asr_kws = @pipeline.get_by_name('asr_kws')
		asr_kws.set_property('hmm', 'pocketsphinx/adapt/cmusphinx-en-us-5.2')
		asr_kws.set_property('mllr', 'pocketsphinx/adapt/mllr_matrix')
		asr_kws.set_property('kws', 'data/keywords.kws')

		bus = @pipeline.bus()
		bus.add_watch do |bus, message|
			case message.type
			when Gst::MessageType::EOS
				loop.quit
			when Gst::MessageType::ERROR
				p message.parse_error
				binding.pry # open console
				loop.quit
			when Gst::MessageType::ELEMENT
				if message.src.name == "asr_kws"
					if message.structure.get_value(:final).value
						keyword_detect(message.structure.get_value(:hypothesis).value, message.structure.get_value(:confidence).value)
					end
				elsif message.src.name == "asr_jsgf"
					if message.structure.get_value(:final).value
						final_result(message.structure.get_value(:hypothesis).value, message.structure.get_value(:confidence).value)
					end
				elsif message.src.name == "cutter"
					if message.structure.get_value(:above).value
						@logger.debug "Start recording.."
					else
						@logger.debug "Stop recording"
					end
				end
			end
			true
		end

		@pipeline.play
	end

	# Enables/Disables the jsgf pipeline branch
	def enable_jsgf(bEnable)
		valve = @pipeline.get_by_name('valve_jsgf')
		valve.set_property("drop", !bEnable)
	end

	# Result of jsgf pipeline branch
	def final_result(hyp, confidence)
		@logger.info "final: " + hyp + " " + confidence.to_s
		enable_jsgf(false)

		# Publish pocketsphinx result as ros message
		msg = Std_msgs::String.new
		msg.data = hyp
		@publisher.publish(msg)
	end

	def keyword_detect(hyp, confidence)
		@logger.debug "Got keyword: " + hyp
		enable_jsgf(true)
	end

	def stop
		@pipeline.stop
	end
end

if __FILE__ == $0
	node = ROS::Node.new('asr_pocketsphinx')
	app = Speak.new(node)
	loop = GLib::MainLoop.new(nil, false)
	begin
		Thread.new {
			loop.run
		}
		node.spin
	rescue Interrupt
	ensure
		app.stop
		node.shutdown
		loop.quit
	end
end
