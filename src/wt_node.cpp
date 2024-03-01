#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <wild_thumper/WildThumperConfig.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <boost/format.hpp>
extern "C" {
#include <i2c/smbus.h>
}

#define I2C_FILE "/dev/i2c-2"
#define WHEEL_DIST 0.248
#define BATTERY_CAPACITY_FULL 2.750 // Ah, NiMH=2.750, LiFePO4=3.100
#define UPDATE_RATE 20.0
#define BOOL_TO_S(b) (b ? "True": "False")
#define PTR_TO_FLOAT(x) ({ \
	uint32_t tmp = __bswap_32(*reinterpret_cast<uint32_t*>(x));	\
	*reinterpret_cast<float*>(&tmp);	\
})


int i2c_write_reg(const uint8_t addr, const uint8_t command, const std::vector<uint8_t> values) {
	int ret, file=-1;

	if ((file = open(I2C_FILE, O_RDWR)) < 0) {
		perror("open");
		goto error;
	}
	if (ioctl(file, I2C_SLAVE, (addr>>1)) < 0) {
		perror("ioctl");
		goto error;
	}

	if ((ret = i2c_smbus_write_i2c_block_data(file, command, values.size(), &values[0])) != 0) {
		perror("i2c_smbus_write_block_data");
		goto error;
	}

	close(file);
	return ret;

error:
	if (file > 0) {
		close(file);
	}
	throw -1;
}

int i2c_read_reg(const uint8_t addr, const uint8_t command, const uint8_t length, uint8_t *values) {
	int ret, file=-1;

	if ((file = open(I2C_FILE, O_RDWR)) < 0) {
		perror("open");
		goto error;
	}
	if (ioctl(file, I2C_SLAVE, (addr>>1)) < 0) {
		perror("ioctl");
		goto error;
	}

	if ((ret = i2c_smbus_read_i2c_block_data(file, command, length, values)) != length) {
		perror("i2c_smbus_read_block_data");
		goto error;
	}

	close(file);
	return ret;

error:
	if (file > 0) {
		close(file);
	}
	throw -1;
}


class WTBase {
	public:
		WTBase(ros::NodeHandle nh, ros::NodeHandle pnh) {
			this->nh = nh;
			this->pnh = pnh;
			enable_odom_tf = pnh.param("enable_odom_tf", true);

			dynamic_reconfigure::Server<wild_thumper::WildThumperConfig>::CallbackType f;
			f = boost::bind(&WTBase::execute_dyn_reconf, this, _1, _2);
			dyn_conf.setCallback(f);

			pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 16);
			pub_diag = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 16);
			pub_range_fwd_left = nh.advertise<sensor_msgs::Range>("range_forward_left", 16);
			pub_range_fwd_right = nh.advertise<sensor_msgs::Range>("range_forward_right", 16);
			pub_range_bwd = nh.advertise<sensor_msgs::Range>("range_backward", 16);
			pub_battery = nh.advertise<sensor_msgs::BatteryState>("battery", 16);
			set_speed(0, 0);
			i2c_write_reg(0x50, 0x90, {1, 1}); // switch direction
			sub_cmd_vel = nh.subscribe("cmd_vel_out", 10, &WTBase::cmdVelReceived, this);
			sub_imu = nh.subscribe("imu", 10, &WTBase::imuReceived, this);
			srv_manipulator_enable = pnh.advertiseService("manipulator_enable", &WTBase::enableManipulator, this);
			ROS_INFO("Init done");
		}

		void run() {
			ros::Rate loop_rate(UPDATE_RATE);
			sleep(3); // wait 3s for ros to register and establish all subscriber connections before sending reset diag
			uint8_t reset_val = get_reset();
			ROS_INFO("Reset Status: 0x%x", reset_val);
			i2c_write_reg(0x50, 0xA4, {1}); // enable Watchdog
			uint32_t sonar_count = 0;
			while (ros::ok()) {
				ROS_DEBUG("Loop alive");
				//print "Watchdog", struct.unpack(">B", i2c_read_reg(0x50, 0xA4, 1))[0]
				//print struct.unpack(">B", i2c_read_reg(0x50, 0xA2, 1))[0] # count test
				get_motor_err();
				get_odom();
				get_power();

				if (sonar_count == 0) {
					get_dist_forward_left();
					update_dist_backward();
					sonar_count+=1;
				} else if (sonar_count == 1) {
					get_dist_backward();
					update_dist_forward_right();
					sonar_count+=1;
				} else if (sonar_count == 2) {
					get_dist_forward_right();
					update_dist_forward_left();
					sonar_count=0;
				}

				if (!cmd_vel.empty()) {
					set_speed(cmd_vel[0], cmd_vel[1]);
					cur_vel[0] = cmd_vel[0]; cur_vel[1] = cmd_vel[1];
					cmd_vel.clear();
				}

				check_docked();

				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		bool isDocked() {
			return bDocked;
		}

		void setVelocity(float trans, float rot) {
			cmd_vel.resize(2);
			cmd_vel[0] = trans;
			cmd_vel[1] = rot;
		}

		bool enableManipulator(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
			ROS_INFO("Set Manipulator: %d", req.data);
			i2c_write_reg(0x52, 0x0f, {req.data});
			res.success = true;
			return true;
		}

	private:
		ros::NodeHandle nh;
		ros::NodeHandle pnh;
		tf::TransformBroadcaster tf_broadcaster;
		dynamic_reconfigure::Server<wild_thumper::WildThumperConfig> dyn_conf;
		ros::Publisher pub_odom;
		ros::Publisher pub_diag;
		ros::Publisher pub_range_fwd_left;
		ros::Publisher pub_range_fwd_right;
		ros::Publisher pub_range_bwd;
		ros::Publisher pub_battery;
		std::vector<float> cmd_vel;
		float cur_vel[2] = {0, 0};
		bool bMotorManual = false;
		ros::Subscriber sub_cmd_vel;
		ros::Subscriber sub_imu;
		ros::ServiceServer srv_manipulator_enable;
		bool bDocked = false;
		bool bDocked_last = false;
		double battery_capacity = BATTERY_CAPACITY_FULL*3600; // unit=As
		bool enable_odom_tf;
		bool bClipRangeSensor;
		double range_sensor_max;
		double range_sensor_fov;
		double odom_covar_xy;
		double odom_covar_angle;
		bool rollover_protect;
		double rollover_protect_limit;
		int16_t rollover_protect_pwm;
		bool bStayDocked;

		void execute_dyn_reconf(wild_thumper::WildThumperConfig &config, uint32_t level) {
			bClipRangeSensor = config.range_sensor_clip;
			range_sensor_max = config.range_sensor_max;
			range_sensor_fov = config.range_sensor_fov;
			odom_covar_xy = config.odom_covar_xy;
			odom_covar_angle = config.odom_covar_angle;
			rollover_protect = config.rollover_protect;
			rollover_protect_limit = config.rollover_protect_limit;
			rollover_protect_pwm = config.rollover_protect_pwm;
			bStayDocked = config.stay_docked;
		}

		void imuReceived(const sensor_msgs::Imu::ConstPtr& msg) {
			bool isDriving = fabs(cur_vel[0]) > 0 || fabs(cur_vel[1]) > 0;
			if (rollover_protect && (isDriving || bMotorManual)) {
				double roll, pitch, yaw;
				tf::Quaternion quat;
				tf::quaternionMsgToTF(msg->orientation, quat);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (pitch > rollover_protect_limit*M_PI/180) {
					bMotorManual = true;
					i2c_write_reg(0x50, 0x1, {0, 0, (uint8_t)(rollover_protect_pwm>>8), (uint8_t)rollover_protect_pwm, 0, 0, (uint8_t)(rollover_protect_pwm>>8), (uint8_t)rollover_protect_pwm});
					ROS_WARN("Running forward rollver protection");
				} else if (pitch < -rollover_protect_limit*M_PI/180) {
					bMotorManual = true;
					i2c_write_reg(0x50, 0x1, {(uint8_t)(-rollover_protect_pwm>>8), (uint8_t)-rollover_protect_pwm, 0, 0, (uint8_t)(-rollover_protect_pwm>>8), (uint8_t)-rollover_protect_pwm, 0, 0});
					ROS_WARN("Running backward rollver protection");
				} else if (bMotorManual) {
					i2c_write_reg(0x50, 0x1, {0, 0, 0, 0, 0, 0, 0, 0});
					bMotorManual = false;
					setVelocity(0, 0);
					ROS_WARN("Rollver protection done");
				}
			}
		}

		uint8_t get_reset() {
			uint8_t buf[1];
			i2c_read_reg(0x50, 0xA0, 1, buf);
			uint8_t reset = buf[0];

			diagnostic_msgs::DiagnosticArray msg;
			msg.header.stamp = ros::Time::now();
			diagnostic_msgs::DiagnosticStatus stat;
			stat.name = "Reset reason";
			stat.level = reset & 0x0c ? diagnostic_msgs::DiagnosticStatus::ERROR : diagnostic_msgs::DiagnosticStatus::OK;
			stat.message = str(boost::format("0x%02x") % reset);

			bool wdrf = reset & (1 << 3);
			if (wdrf) ROS_INFO("Watchdog Reset");
			bool borf = reset & (1 << 2);
			if (borf) ROS_INFO("Brown-out Reset Flag");
			bool extrf = reset & (1 << 1);
			if (extrf) ROS_INFO("External Reset Flag");
			bool porf = reset & (1 << 0);
			if (porf) ROS_INFO("Power-on Reset Flag");

			diagnostic_msgs::KeyValue kvWdrf, kvBorf, kvExtrf, kvPorf;
			kvWdrf.key = "Watchdog Reset Flag";
			kvWdrf.value = BOOL_TO_S(wdrf);
			stat.values.push_back(kvWdrf);
			kvBorf.key = "Brown-out Reset Flag";
			kvBorf.value = BOOL_TO_S(borf);
			stat.values.push_back(kvBorf);
			kvExtrf.key = "External Reset Flag";
			kvExtrf.value = BOOL_TO_S(extrf);
			stat.values.push_back(kvExtrf);
			kvPorf.key = "Power-on Reset Flag";
			kvPorf.value = BOOL_TO_S(porf);
			stat.values.push_back(kvPorf);

			msg.status.push_back(stat);
			pub_diag.publish(msg);
			return reset;
		}

		void get_motor_err() {
			uint8_t buf[1];
			i2c_read_reg(0x50, 0xA1, 1, buf);
			uint8_t err = buf[0];

			diagnostic_msgs::DiagnosticArray msg;
			msg.header.stamp = ros::Time::now();
			diagnostic_msgs::DiagnosticStatus stat;
			stat.name = "Motor: Error Status";
			stat.level = err ? diagnostic_msgs::DiagnosticStatus::ERROR : diagnostic_msgs::DiagnosticStatus::OK;
			stat.message = str(boost::format("0x%02x") % err);

			// Diag
			diagnostic_msgs::KeyValue kvAftLeft, kvFrontLeft, kvAftRight, kvFrontRight;
			kvAftLeft.key = "aft left diag";
			kvAftLeft.value = BOOL_TO_S((bool)(err & (1 << 0)));
			stat.values.push_back(kvAftLeft);
			kvFrontLeft.key = "front left diag";
			kvFrontLeft.value = BOOL_TO_S((bool)(err & (1 << 1)));
			stat.values.push_back(kvFrontLeft);
			kvAftRight.key = "aft right diag";
			kvAftRight.value = BOOL_TO_S((bool)(err & (1 << 2)));
			stat.values.push_back(kvAftRight);
			kvFrontRight.key = "front right diag";
			kvFrontRight.value = BOOL_TO_S((bool)(err & (1 << 3)));
			stat.values.push_back(kvFrontRight);
			// Stall
			diagnostic_msgs::KeyValue kvAftLeft2, kvFrontLeft2, kvAftRight2, kvFrontRight2;
			kvAftLeft2.key = "aft left stall";
			kvAftLeft2.value = BOOL_TO_S((bool)(err & (1 << 4)));
			stat.values.push_back(kvAftLeft2);
			kvFrontLeft2.key = "front left stall";
			kvFrontLeft2.value = BOOL_TO_S((bool)(err & (1 << 5)));
			stat.values.push_back(kvFrontLeft2);
			kvAftRight2.key = "aft right stall";
			kvAftRight2.value = BOOL_TO_S((bool)(err & (1 << 6)));
			stat.values.push_back(kvAftRight2);
			kvFrontRight2.key = "front right stall";
			kvFrontRight2.value = BOOL_TO_S((bool)(err & (1 << 7)));
			stat.values.push_back(kvFrontRight2);

			msg.status.push_back(stat);
			pub_diag.publish(msg);
		}

		void get_power() {
			uint8_t buf[2];
			i2c_read_reg(0x52, 0x09, 2, buf);
			double volt = __bswap_16(*(int16_t*)buf)/100.0;
			i2c_read_reg(0x52, 0x0D, 2, buf);
			double current = __bswap_16(*(int16_t*)buf)/1000.0;

			diagnostic_msgs::DiagnosticArray msg;
			msg.header.stamp = ros::Time::now();
			diagnostic_msgs::DiagnosticStatus stat;
			stat.name = "Power";
			stat.level = volt < 7 || current > 5 ? diagnostic_msgs::DiagnosticStatus::ERROR : diagnostic_msgs::DiagnosticStatus::OK;
			stat.message = str(boost::format("%.2fV, %.2fA") % volt % current);

			msg.status.push_back(stat);
			pub_diag.publish(msg);

			if (volt < 7) {
				ROS_ERROR_THROTTLE(10, "Voltage critical: %.2fV", volt);
			}

			bDocked = volt > 10.1;
			update_capacity(volt, current);

			if (pub_battery.getNumSubscribers()) {
				sensor_msgs::BatteryState batmsg;
				batmsg.header.stamp = ros::Time::now();
				batmsg.voltage = volt;
				batmsg.current = current;
				batmsg.charge = battery_capacity/3600.0;
				batmsg.capacity = NAN;
				batmsg.design_capacity = BATTERY_CAPACITY_FULL;
				batmsg.percentage = batmsg.charge/batmsg.design_capacity;
				batmsg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
				batmsg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
				batmsg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
				batmsg.present = true;
				pub_battery.publish(batmsg);
			}
		}

		void update_capacity(double volt, double current) {
			if (bDocked)
				battery_capacity = BATTERY_CAPACITY_FULL*3600;
			else
				battery_capacity -= current/UPDATE_RATE;
		}

		void get_odom() {
			uint8_t buf[20];
			i2c_read_reg(0x50, 0x38, 20, buf);
			float speed_trans = PTR_TO_FLOAT(buf+0);
			float speed_rot = PTR_TO_FLOAT(buf+4);
			float posx = PTR_TO_FLOAT(buf+8);
			float posy = PTR_TO_FLOAT(buf+12);
			float angle = PTR_TO_FLOAT(buf+16);
			ros::Time current_time = ros::Time::now();

			// since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);

			// first, we'll publish the transform over tf
			if (enable_odom_tf) {
				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";
				odom_trans.transform.translation.x = posx;
				odom_trans.transform.translation.y = posy;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
				tf_broadcaster.sendTransform(odom_trans);
			}

			// next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			// set the position
			odom.pose.pose.position.x = posx;
			odom.pose.pose.position.y = posy;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance[0] = odom_covar_xy; // x
			odom.pose.covariance[7] = odom_covar_xy; // y
			odom.pose.covariance[14] = odom_covar_xy; // z
			odom.pose.covariance[21] = odom_covar_angle; // rotation about X axis
			odom.pose.covariance[28] = odom_covar_angle; // rotation about Y axis
			odom.pose.covariance[35] = odom_covar_angle; // rotation about Z axis

			// set the velocity
			odom.child_frame_id = "base_footprint";
			odom.twist.twist.linear.x = speed_trans;
			odom.twist.twist.linear.y = 0.0;
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.z = speed_rot;
			odom.twist.covariance[0] = odom_covar_xy; // x
			odom.twist.covariance[7] = odom_covar_xy; // y
			odom.twist.covariance[14] = odom_covar_xy; // z
			odom.twist.covariance[21] = odom_covar_angle; // rotation about X axis
			odom.twist.covariance[28] = odom_covar_angle; // rotation about Y axis
			odom.twist.covariance[35] = odom_covar_angle; // rotation about Z axis

			// publish the message
			pub_odom.publish(odom);
		}

		void set_speed(float trans, float rot) {
			std::vector<uint8_t> buf;
			buf.push_back((*(uint32_t*)&trans)>>24);
			buf.push_back((*(uint32_t*)&trans)>>16);
			buf.push_back((*(uint32_t*)&trans)>>8);
			buf.push_back((*(uint32_t*)&trans)>>0);
			buf.push_back((*(uint32_t*)&rot)>>24);
			buf.push_back((*(uint32_t*)&rot)>>16);
			buf.push_back((*(uint32_t*)&rot)>>8);
			buf.push_back((*(uint32_t*)&rot)>>0);
			i2c_write_reg(0x50, 0x50, buf);
		}

		void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& msg) {
			if (!bMotorManual) {
				ROS_DEBUG("Set new cmd_vel: %.2f %.2f", msg->linear.x, msg->angular.z);
				setVelocity(msg->linear.x, msg->angular.z); // commit speed on next update cycle
				ROS_DEBUG("Set new cmd_vel done");
			}
		}

		void start_dist_srf(uint8_t num) {
			i2c_write_reg(0x52, num, {});
		}

		float read_dist_srf(uint8_t num) {
			uint8_t buf[2];
			i2c_read_reg(0x52, num, 2, buf);
			return __bswap_16(*(uint16_t*)buf)/1000.0;
		}

		void send_range(ros::Publisher &pub, std::string frame_id, uint8_t typ, double dist, double min_range, double max_range, double fov_deg) {
			if (bClipRangeSensor && dist > max_range) {
				dist = max_range;
			}
			sensor_msgs::Range msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = frame_id;
			msg.radiation_type = typ;
			msg.field_of_view = fov_deg*M_PI/180;
			msg.min_range = min_range;
			msg.max_range = max_range;
			msg.range = dist;
			pub.publish(msg);
		}

		void get_dist_forward_left() {
			if (pub_range_fwd_left.getNumSubscribers() > 0) {
				float dist = read_dist_srf(0x15);
				send_range(pub_range_fwd_left, "sonar_forward_left", sensor_msgs::Range::ULTRASOUND, dist, 0.04, range_sensor_max, range_sensor_fov);
			}
		}

		void update_dist_forward_left() {
			if (pub_range_fwd_left.getNumSubscribers() > 0) {
				start_dist_srf(0x5);
			}
		}

		void get_dist_backward() {
			if (pub_range_bwd.getNumSubscribers() > 0) {
				float dist = read_dist_srf(0x17);
				send_range(pub_range_bwd, "sonar_backward", sensor_msgs::Range::ULTRASOUND, dist, 0.04, range_sensor_max, range_sensor_fov);
			}
		}

		void update_dist_backward() {
			if (pub_range_bwd.getNumSubscribers() > 0) {
				start_dist_srf(0x7);
			}
		}

		void get_dist_forward_right() {
			if (pub_range_fwd_right.getNumSubscribers() > 0) {
				float dist = read_dist_srf(0x19);
				send_range(pub_range_fwd_right, "sonar_forward_right", sensor_msgs::Range::ULTRASOUND, dist, 0.04, range_sensor_max, range_sensor_fov);
			}
		}

		void update_dist_forward_right() {
			if (pub_range_fwd_right.getNumSubscribers() > 0) {
				start_dist_srf(0xb);
			}
		}

		void check_docked() {
			if (bDocked && !bDocked_last) {
				ROS_INFO("Docking event");
			} else if (!bDocked && bDocked_last) {
				bool stopped = cur_vel[0] == 0 && cur_vel[1] == 0;
				if (!bStayDocked || !stopped) {
					ROS_INFO("Undocking event");
				} else {
					ROS_INFO("Undocking event..redocking");
					pthread_t tid;
					pthread_create(&tid, NULL, &WTBase::redock, this);
				}
			}

			bDocked_last = bDocked;
		}

		static void *redock(void *context) {
			WTBase *This = (WTBase *)context;
			This->setVelocity(-0.1, 0);
			for (int i=0; i<100; i++) {
				if (This->isDocked())
					break;
				usleep(10*1000);
			}
			This->setVelocity(0, 0);
			if (This->isDocked()) {
				ROS_INFO("Redocking done");
			} else {
				ROS_ERROR("Redocking failed");
			}

			pthread_exit((void *) 0);
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "wild_thumper");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	WTBase robot(nh, pnh);
	robot.run();
	return 0;
}
