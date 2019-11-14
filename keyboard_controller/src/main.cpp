#include <iostream>
#include <algorithm>
#include <cstdint>
#include <memory>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>

enum AppState { RUNNING, QUIT };

enum ControlState { IDLE, ACCELERATING, DECCELERATING};
enum TransmissionState {FORWARD, REVERSE };

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}



struct VehicleTeleopState
{
	double acceleration;
	double current_speed;
	double last_measurement;
	double wheel_angle;
	AppState appstate = RUNNING;
	ControlState control_state = IDLE;
	TransmissionState transmission_state = FORWARD;
};

struct VehicleTeleopConfig
{
	double update_freq;
};

constexpr double MAX_ACCELERATION = 3.0;
constexpr double MAX_BRAKE = 4*MAX_ACCELERATION;
constexpr double D_ACCELERATION_RATE = 0.1;
constexpr double D_BRAKE_RATE = 0.3;
constexpr double DECCELERATION_RATE = 5.5;
constexpr double MAX_WHEEL_ANG = 32.0*M_PI/180.0;
constexpr double D_WHEEL_ANG = MAX_WHEEL_ANG/7.0;

class KeyboardApp {
private:
	// Configuration
	std::unique_ptr<VehicleTeleopConfig> teleop_config;
	// Teleoperation state
	std::unique_ptr<VehicleTeleopState> teleop_state;
	//

	SDL_Event e;
	//The window we'll be rendering to
	SDL_Window* gWindow;

	//The surface contained by the window
	SDL_Surface* gScreenSurface;

	//The image we will load and show on the screen
	SDL_Surface* gHelloWorld;
	// ROS specific stuff
	ros::NodeHandle& nh;
	ros::Timer ctrl_timer;
	// Publisher
	autoware_msgs::ControlCommandStamped msg_ctrl_cmd;
	ros::Publisher pub_ctrl_cmd;
public:
	KeyboardApp(ros::NodeHandle& nh): nh(nh), teleop_state(new VehicleTeleopState())
	{

	}

	~KeyboardApp()
	{
		teleop_state.reset();
	}

	bool init(double update_freq)
	{
		teleop_config = std::unique_ptr<VehicleTeleopConfig>(new VehicleTeleopConfig{update_freq});
		if (SDL_Init(SDL_INIT_VIDEO) < 0)
		{
			return false;
		}
		gWindow = SDL_CreateWindow("Keyboard controller", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 200, SDL_WINDOW_SHOWN);
		gScreenSurface = SDL_GetWindowSurface(gWindow);
		SDL_FillRect( gScreenSurface, NULL, SDL_MapRGB( gScreenSurface->format, 0xFF, 0xFF, 0xFF ) );

		teleop_state->last_measurement = ros::Time::now().toSec();
		pub_ctrl_cmd = nh.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 10);
		ctrl_timer = nh.createTimer(ros::Duration(1.0/teleop_config->update_freq), &KeyboardApp::cbCtrlTimer, this);

		return true;
	}

	void cbCtrlTimer(const ros::TimerEvent& event)
	{
		teleop_state->current_speed += teleop_state->acceleration * 1.0/teleop_config->update_freq;
		switch(teleop_state->control_state)
		{
		case DECCELERATING:
		{
			teleop_state->current_speed -= 1.5 * 1.0/teleop_config->update_freq;
			if (teleop_state->current_speed <= 0.0)
			{
				teleop_state->current_speed = 0;
				teleop_state->control_state = IDLE;
			}
			break;
		}
		}
		msg_ctrl_cmd.header.stamp = ros::Time::now();
		msg_ctrl_cmd.cmd.linear_velocity = teleop_state->current_speed;
		pub_ctrl_cmd.publish(msg_ctrl_cmd);
	}

	void run()
	{
		while (teleop_state->appstate!=QUIT)
		{
			while(SDL_PollEvent(&e)!= 0)
			{
				if (e.type == SDL_QUIT)
				{
					teleop_state->appstate = QUIT;
				}
				else if(e.type== SDL_KEYDOWN)
				{
					switch(e.key.keysym.sym)
					{
					case SDLK_UP:
					case SDLK_w:{
						teleop_state->control_state = ACCELERATING;
						teleop_state->acceleration = clamp(
								teleop_state->acceleration + D_ACCELERATION_RATE,
								0.0,
								MAX_ACCELERATION);
						msg_ctrl_cmd.cmd.linear_acceleration = teleop_state->acceleration;
						teleop_state->last_measurement = ros::Time::now().toSec();
						break;
					}
					case SDLK_DOWN: {
						teleop_state->control_state = ACCELERATING;
						if (teleop_state->current_speed >= 0.0)
						{
							teleop_state->acceleration = clamp(
									teleop_state->acceleration - D_BRAKE_RATE,
									-MAX_BRAKE,
									0.0);
						}
						else
						{
							teleop_state->acceleration = 0.0;
							teleop_state->current_speed = 0.0;
						}
						msg_ctrl_cmd.cmd.linear_acceleration = teleop_state->acceleration;
						teleop_state->last_measurement = ros::Time::now().toSec();
						break;
						break;
					}

					case SDLK_LEFT:{
						teleop_state->wheel_angle = clamp(teleop_state->wheel_angle + D_WHEEL_ANG,
								-MAX_WHEEL_ANG,
								MAX_WHEEL_ANG);
						msg_ctrl_cmd.cmd.steering_angle = teleop_state->wheel_angle;
						break;
					}
					case SDLK_RIGHT:{
						teleop_state->wheel_angle = clamp(teleop_state->wheel_angle - D_WHEEL_ANG,
								-MAX_WHEEL_ANG,
								MAX_WHEEL_ANG);
						msg_ctrl_cmd.cmd.steering_angle = teleop_state->wheel_angle;
						break;
					}
					}
				}
				else if (e.type == SDL_KEYUP)
				{
					switch(e.key.keysym.sym)
					{
					case SDLK_UP:
					case SDLK_w:{
						teleop_state->acceleration = 0.0;
						msg_ctrl_cmd.cmd.linear_acceleration = teleop_state->acceleration;
						teleop_state->last_measurement = ros::Time::now().toSec();
						teleop_state->control_state = DECCELERATING;
						break;
					}
					case SDLK_LEFT:
					case SDLK_RIGHT:{
						teleop_state->wheel_angle = 0.0;
						msg_ctrl_cmd.cmd.steering_angle = 0.0;
						break;
					}
					}
				}

			}
			SDL_UpdateWindowSurface(gWindow);
		}
	}

	void teardown()
	{
		SDL_DestroyWindow(gWindow);
		SDL_Quit();
	}



};





int main(int argc, char** argv)
{
	ros::init(argc,argv, "szenergy_keyboard_controller");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	KeyboardApp keyboardapp(nh);
	double update_freq = 100.0;
	if (!private_nh.getParam("update_freq", update_freq))
	{
		ROS_WARN("No update frequency specified, using default (100 Hz)");
	}

	if (keyboardapp.init(update_freq))
	{
		ros::AsyncSpinner spinner(4);
		spinner.start();
		keyboardapp.run();
		keyboardapp.teardown();
		return 0;
	}
	else
	{
		std::cerr << "Could not initialize SDL, quitting" << '\n';

		return -1;
	}
}
