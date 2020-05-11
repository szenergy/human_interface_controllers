#include <iostream>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <cstdio>
#include <thread>
#include <chrono>

#include <ros/ros.h>


#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>

#include <linux/input.h>

#include <ncurses.h>

#include <autoware_msgs/ControlCommandStamped.h>

#include <szelectricity_common/MathCommon.hpp>

namespace szenergy
{

constexpr double EPS_CONTROL_LINVEL_ERROR = 0.04;
constexpr double EPS_CONTROL_ANG_ERROR = 0.005;

constexpr const char* KEYBOARD_MONITOR_INPUT_FIFO_NAME = "KeyboardMonitorInputFifo";

enum class ControllerState {IDLE, CONTROL, STOPPED};

enum class SteerState {IDLE, STEER};


class KeyboardController
{
private:
	ros::NodeHandle nh;
	const double ACCELERATION_RATE = 0.7;
	const double ANG_RATE = 0.2;
	const double BRAKE_RATE = 0.2;
	const double DECAY_VEL_RATE = 0.4;
	const double DECAY_ANG_RATE = 0.05;
	const double LIMIT_WHEEL_ANGLE = 32*M_PI/180.0;

	ControllerState cstate;
	SteerState steerstate;
	int input_fifo_filestream;
	std::thread keyboard_thread;
	// Starting periodical publish of control message
	ros::Timer timer;
	ros::Timer timer_decay;
	ros::Publisher pub_cmd;
	autoware_msgs::ControlCommandStamped cmd;
	ros::Time t_prev_ros;
public:
	KeyboardController(ros::NodeHandle& nh):
		nh(nh),
		input_fifo_filestream(-1),
		steerstate(SteerState::IDLE),
		cstate(ControllerState::IDLE){}


	bool isRunning()
	{
		return cstate!=ControllerState::STOPPED;
	}

	void keyboard_thread_worker()
	{
		ROS_INFO("Started keyboard reader");
		static std::chrono::steady_clock::time_point t_prev = std::chrono::steady_clock::now();
		int c;
		printw("Car-like keyboard control\n");
		printw("-------------------------\n");
		printw("    /\\   :W\n");
		printw("A:<    > :D\n");
		printw("    \\/   :S\n");
		printw("\nQ: Exit");
		while(cstate!=ControllerState::STOPPED)
		{
			c = getch();
			double dt = 0.0;
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
			if (cstate == ControllerState::CONTROL || steerstate == SteerState::STEER)
			{
				std::chrono::duration<double> d = t1 - t_prev;
				dt = szenergy::ThresholdMax(d.count(), 1.0);
			}
			t_prev_ros = ros::Time::now();
			// Handle incoming keys
			switch(c)
			{
			case KEY_UP:
			case 'W':
			case 'w':
			{
				cstate = ControllerState::CONTROL;
				cmd.cmd.linear_velocity += ACCELERATION_RATE*dt;
				break;
			}
			case KEY_DOWN:
			case 'S':
			case 's':
			{
				cstate = ControllerState::CONTROL;
				cmd.cmd.linear_velocity -= ACCELERATION_RATE*dt;
				break;
			}
			case KEY_LEFT:
			case 'a':
			case 'A':
			{
				steerstate = SteerState::STEER;
				cmd.cmd.steering_angle += ANG_RATE*dt;
				break;
			}
			case KEY_RIGHT:
			case 'd':
			case 'D':
			{
				steerstate = SteerState::STEER;
				cmd.cmd.steering_angle -= ANG_RATE*dt;
				break;
			}
			case 'q':
			case 'Q':
			{
				timer.stop();
				timer_decay.stop();
				cmd.cmd.linear_velocity = 0.0;
				cmd.cmd.steering_angle = 0.0;
				cstate = ControllerState::STOPPED;				
				break;
			}
			default:
			{
				cstate = ControllerState::IDLE;
				steerstate = SteerState::IDLE;
			}
			}
			cmd.cmd.steering_angle = szenergy::Clamp(cmd.cmd.steering_angle, -LIMIT_WHEEL_ANGLE, LIMIT_WHEEL_ANGLE);
			t_prev = t1;
		}
	}

	void cbDecay(const ros::TimerEvent& event)
	{
		double dt = event.current_real.toSec() - event.last_real.toSec();
		// Decay velocity
		switch(cstate)
		{
		case ControllerState::IDLE:
		{
			if (std::abs(cmd.cmd.linear_velocity) > EPS_CONTROL_LINVEL_ERROR)
			{
				cmd.cmd.linear_velocity -= szenergy::Sgn(cmd.cmd.linear_velocity) * dt * DECAY_VEL_RATE;
			}
			else
			{
				cmd.cmd.linear_velocity = 0;
			}
			break;
		}
		case ControllerState::CONTROL:
		{
			if (event.current_real.toSec() - t_prev_ros.toSec() > 0.2)
			{
				cstate = ControllerState::IDLE;
			}
			break;
		}
		}
		// Steer decay
		switch(steerstate)
		{
		case SteerState::IDLE:
		{
			if (std::abs(cmd.cmd.steering_angle) > EPS_CONTROL_ANG_ERROR)
			{
				cmd.cmd.steering_angle -= szenergy::Sgn(cmd.cmd.steering_angle) * dt * DECAY_ANG_RATE;
			}
			else
			{
				cmd.cmd.steering_angle = 0;
			}
			break;
		}
		case SteerState::STEER:
		{
			if (event.current_real.toSec() - t_prev_ros.toSec() > 0.2)
			{
				steerstate = SteerState::IDLE;
			}
			break;
		}
		}
	}

	void cbPubTimer(const ros::TimerEvent& event)
	{
		cmd.header.stamp = ros::Time::now();
		pub_cmd.publish(cmd);
	}

	void initialize()
	{
		// Initialize NCURSES
		initscr();
		noecho();

		pub_cmd = nh.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 10);
		timer = nh.createTimer(ros::Duration(0.02), &KeyboardController::cbPubTimer, this);
		timer_decay = nh.createTimer(ros::Duration(0.02), &KeyboardController::cbDecay, this);
		keyboard_thread = std::thread(&KeyboardController::keyboard_thread_worker, this);
		timer.start();
		ROS_INFO("Started threads");

	}

	void stop()
	{
		timer.stop();
		clrtoeol();
		endwin();
		cstate = ControllerState::STOPPED;
		ROS_INFO("Stopped threads");
	}

};

}

int main(int argc, char** argv)
{
	ros::init(argc,argv, "szenergy_keyboard_controller");
	ros::NodeHandle nh;
	szenergy::KeyboardController controller(nh);
	controller.initialize();
	while(ros::ok() && controller.isRunning())
	{
		ros::spinOnce();
	}

	controller.stop();


	return 0;
}
