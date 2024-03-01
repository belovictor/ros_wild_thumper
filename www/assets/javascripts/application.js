function init() {
	var ros = new ROSLIB.Ros();
	connect();
	var isDragging = false;

	function connect() {
		ros.connect('ws://'+robothostname+':9090');
	}

	ros.on('connection', function() {
		information.alerts.push({message: "Connected to websocket server.", success: true});
	});

	ros.on('error', function(error) {
		information.alerts.push({message: "Error connecting to websocket server.", danger: true});
	});

	ros.on('close', function() {
		information.alerts.push({message: "Connection to websocket server closed.", info: true});
		setTimeout(function() {
			connect();
		}, 2000);
	});

	//tfClient.subscribe('base_link', function(tf) {
	//	var now = new Date();
	//	$("#pose").text(now.toLocaleTimeString() + ": (" + tf.translation.x + ", " + tf.translation.y + ")");

	//	robotMarker.x = tf.translation.x;
	//	robotMarker.y = -tf.translation.y;
	//	robotMarker.rotation = viewer2D.scene.rosQuaternionToGlobalTheta(tf.rotation);
	//});
	var poseTopic = new ROSLIB.Topic({ros: ros, name: '/robot_pose', messageType: 'geometry_msgs/Pose'});
	var sensorTopic = new ROSLIB.Topic({ros: ros, name: '/sensors', messageType: 'wild_thumper/Sensor'});
	var batteryTopic = new ROSLIB.Topic({ros: ros, name: '/battery', messageType: 'sensor_msgs/BatteryState'});
	var ledStripeTopic = new ROSLIB.Topic({ros: ros, name: '/led_stripe', messageType: 'wild_thumper/LedStripe'});
	var cmdVelTopic = new ROSLIB.Topic({ros: ros, name: '/teleop/cmd_vel', messageType: 'geometry_msgs/Twist'});

	poseTopic.subscribe(function(message) {
		var now = new Date();
		$("#pose").text(now.toLocaleTimeString() + ": (" + message.position.x + ", " + message.position.y + ")");
	});

	sensorTopic.subscribe(function(message) {
		sensors.light = message.light;
		sensors.temp = message.temp.toFixed(1);
		sensors.humidity = message.humidity;
		sensors.pressure = message.pressure.toFixed(1);
		sensors.co = message.co;
	});

	batteryTopic.subscribe(function(message) {
		power.voltage = message.voltage.toFixed(1);
		power.current = message.current.toFixed(1);
	});

	viewer2D = new ROS2D.Viewer({
		divID: 'map',
		width: 640,
		height: 480,
		background: "#efefef"
	});
	$('#map')
	.bind('mousewheel', function(e) {
		if (e.originalEvent.wheelDelta/120 > 0) {
			viewer2D.scaleToDimensions(10, 10)
		} else {
			viewer2D.scaleToDimensions(5, 5)
		}
	})
	.mousedown(function() {
		isDragging = true;
		mousePreX = undefined;
		mousePreY = undefined;
	})
	.mouseup(function() {
		isDragging = false;
	})
	.mousemove(function(event) {
		if (isDragging) {
			if (mousePreX != undefined && mousePreY != undefined) {
				var diffX = event.pageX - mousePreX;
				var diffY = event.pageY - mousePreY;
				console.log("Moving viewer2D by " + diffX + ", " + diffY);
				viewer2D.shift(diffX, diffY);
			}
			mousePreX = event.pageX;
			mousePreY = event.pageY;
		}
	});

	// Setup the nav client.
	NAV2D.OccupancyGridClientNav({
		ros : ros,
		rootObject : viewer2D.scene,
		viewer : viewer2D,
		serverName : '/move_base'
	});

	// Initialize the teleop.
	teleop = new KEYBOARDTELEOP.Teleop({
		ros : ros,
		topic : '/teleop/cmd_vel'
	});

	// Create a UI slider using JQuery UI.
	$('#speed-slider').slider({
		range : 'min',
		min : 0.10,
		max : 1.0,
		step : 0.05,
		value : 0.5,
		slide : function(event, ui) {
			// Change the speed label.
			speed_label.speed = ui.value;
			// Scale the speed.
			teleop.scale = (ui.value * 2);
		}
	});

	// Set the initial speed.
	teleop.scale = ($('#speed-slider').slider('value') * 2);

	new Vue({
		el: '#lights',
		data: {
			front_row: ["[0]", "[1]", "[2]", "[3]"],
			top_row: ["[7]", "[6]", "[5]", "[4]"],
			aft_row: ["[8]", "[9]", "[10]", "[11]"],
			bottom_left_row: ["[14]", "[15]"],
			bottom_right_row: ["[13]", "[12]"],
		}
	})
	$('.led_color').minicolors({
		control: 'wheel',
		format: 'rgb',
		defaultValue: '#000000',
		change: function(value) {
			var rgb = $(this).minicolors('rgbObject');
			var nums = jQuery.parseJSON($(this).prop("name"));
			var msg = new ROSLIB.Message({
				leds: []
			});
			jQuery.each(nums, function(i, num) {
				msg["leds"].push({
					num: num,
					red: parseInt(rgb.r*127/255),
					green: parseInt(rgb.g*127/255),
					blue: parseInt(rgb.b*127/255)
				})
			});
			ledStripeTopic.publish(msg);
		}
	});

	function setSpeed(trans, rot) {
		var msg = new ROSLIB.Message({
			linear: {
				x : trans,
				y : 0,
				z : 0
			},
			angular: {
				x : 0,
				y : 0,
				z : rot
			},
		});
		cmdVelTopic.publish(msg);
	}

	$('.cmd_vel_circle')
	.bind('mousedown touchstart', function(e) {
		isDragging = true;
	})
	.bind('mouseup touchend mouseleave', function(e) {
		isDragging = false;
		setSpeed(0, 0);
	})
	.bind('mousemove touchmove', function(e) {
		if (isDragging) {
			// absolute click position
			var X,Y;
			if (e.originalEvent.touches) {
				X = e.originalEvent.touches[0].pageX;
				Y = e.originalEvent.touches[0].pageY;
			} else {
				X = e.pageX;
				Y = e.pageY;
			}
			// relative click position
			var Xrel = X - this.getBoundingClientRect().left - $(this).width()/2;
			var Yrel = Y - this.getBoundingClientRect().top - $(this).height()/2;
			// scale to -1..+1
			var trans = -Yrel / ($(this).height()/2);
			var rot = -Xrel / ($(this).width()/2);
			setSpeed(trans*$("#scale_trans").val(), rot*$("#scale_rot").val());
		}
	});

	information = new Vue({
		el: '#information',
		data: {
			alerts: [],
		},
		methods: {
			classObject: function(id) {
				return {
					"alert-success": this.alerts[id].success,
					"alert-danger": this.alerts[id].danger,
					"alert-info": this.alerts[id].info
				}
			}
		}
	})

	sensors = new Vue({
		el: '#sensors',
		data: {light: '', temp: '', humidity: '', pressure: '', co: ''}
	})

	power = new Vue({
		el: '#power',
		data: {
			voltage: '',
			current: '',
		}
	})

	speed_label = new Vue({
		el: '#speed-label',
		data: {
			speed: $('#speed-slider').slider('value'),
		}
	})

	$(".imagelink").on('click',function(){
		// reload
		$("img").attr("src", $("img").attr("src"))
	});

	$("input[type='number']").spinner();
	$("#usb_cam").attr("src", "http://"+robothostname+":8080/stream?topic=/camera/color/image_raw");
}

Vue.component('input-value', {
	template: '#input-value-template',
	props: ['value', 'label', 'unit']
})

Vue.component('input-led', {
	template: '#input-led-template',
	props: ['name', 'label']
})
