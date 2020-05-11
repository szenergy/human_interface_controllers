#ifndef SZENERGY_LOGITECH_TELEOP_KEYBOARD
#define SZENERGY_LOGITECH_TELEOP_KEYBOARD

#include <ncurses.h>
#include <curses.h>

#include <stdio.h>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include "abstract_teleoperation.hpp"

const double GAS_PEDAL_MAX = 65535/2.0;
const double GAS_PEDAL_RANGE = 65535;
const double STEER_PEDAL_LIMIT = 65535/2.0;
const double STEER_PEDAL_RANGE = 65535;
const double STEER_THROTTLE_EPSILON_MIN = 1e-3;
const double STEER_THROTTLE_EPSILON_MAX = 0.99000;

/**
 * @brief: A class which inputs a steering device and controls vehicles
 * through ROS topics
 * */
class LogitechTeleop: public SzenergyRosTeleoperation
{
private:
    const std::string input_dev_name;
    int joy_fd;
    int num_of_axis; /// Number of axes
    int num_of_buttons; /// Number of buttons
    int* axis; /// Array of axes
    char *button;
    char name_of_joystick[80];
    js_event js;

    std::thread read_loop; /// Looping background thread to read raw values from input steer
    std::thread ui_loop; /// Thread for ncurses UI
    std::thread publish_loop; /// Thread to publish ROS messages

    
protected:
    
    
public:
    LogitechTeleop(const std::string device_steer, 
            const szenergy::TeleopState& state,
            std::shared_ptr<ros::NodeHandle> nh): 
        SzenergyRosTeleoperation(state, nh),
        input_dev_name(device_steer),
        axis(NULL),
        num_of_axis(0),
        num_of_buttons(0),
        button(NULL),
        joy_fd(0), // It shouldn't cause anything to initialize it
        name_of_joystick("") // ditto
        
    {
        
    }

    /**
     * @brief: helper function which inputs the re-mapped values 
     * from steer input
     * */
    void UpdateCmd(const double& gaspedal,
        const double& steer_angle,
        const char& brake_state)
    {
        teleop_pub_state.msg_steer.data = szenergy::Clamp(steer_angle, 
                control_state.steerMin, control_state.steerMax
            );        
        teleop_pub_state.msg_torque.data = szenergy::CutoffRange(
            szenergy::Clamp(gaspedal, 0.0, 1.0), 
            STEER_THROTTLE_EPSILON_MIN, 0.0,
            STEER_THROTTLE_EPSILON_MAX, 1.0);
        // TODO: this part stinks as hell
        double linvel = szenergy::CutoffRange(
            szenergy::Clamp(gaspedal, 0.0, 1.0), 
            STEER_THROTTLE_EPSILON_MIN, 0.0,
            STEER_THROTTLE_EPSILON_MAX, 10.0);
        double steerangle = szenergy::Clamp(steer_angle, 
            control_state.steerMin, control_state.steerMax
        );
        cmd_vel_msg.twist.linear.x = linvel;
        cmd_vel_msg.twist.angular.z = steerangle;
        cmd_autoware_msg.cmd.linear_velocity = linvel;
        cmd_autoware_msg.cmd.steering_angle = steerangle;
        if (brake_state){
            teleop_pub_state.msg_brake.data = true;
            cmd_vel_msg.twist.linear.x = -cmd_vel_msg.twist.linear.x;
        }
        else{
            teleop_pub_state.msg_brake.data = false;            
        }
        
    }

    /**
     * @brief: Update cycle which runs in the background
     * */
    void UpdateCycle(){
        ros::Rate r(control_state.update_rate);
        char brake_state;
        while(ros::ok())
        {
            timestamp = ros::Time::now();
            control_state.steer_angle = (double)axis[0]/(double)STEER_PEDAL_RANGE*2*M_PI;
            control_state.gaspedal = (GAS_PEDAL_MAX-(double)axis[1])/GAS_PEDAL_RANGE; 
            brake_state = button[1];
            UpdateCmd(control_state.gaspedal, control_state.steer_angle, brake_state);
            PublishCmd();
            std::this_thread::yield();
            r.sleep();
        }
        
    }

    /**
     * @brief: Initialize the steering input
     * */
    bool SteerInit(){
        if ((joy_fd=open(input_dev_name.c_str(), O_RDONLY))==-1)
        {
            printf ("Joystick is unavailable\n");
            return false;
        }
        ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
        ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
        ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

        axis = (int *) calloc( num_of_axis, sizeof( int ) );
        button = (char *) calloc( num_of_buttons, sizeof( char ) );

        printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
            , name_of_joystick
            , num_of_axis
            , num_of_buttons );
        fcntl( joy_fd, F_SETFL, O_NONBLOCK );

        

        read_loop = std::thread(&LogitechTeleop::SteerLoop, this);
        read_loop.detach();
        return true;
    }

    void UiInit(){
        initscr();
        noecho();
        halfdelay(TRUE);
    }

    bool InitSteer()
    {
        if (!SteerInit())
        {
            return false;
        }
        publish_loop = std::thread(&LogitechTeleop::UpdateCycle, this);
        publish_loop.detach();
        return true;
    }

    void UiLoop(){
        move(0,0);
        printw("Controller: %s", name_of_joystick);
        move(1,0);
        printw("Steer angle: ");
        move(2,0);
        printw("Torque: ");
        move(3,0);
        printw("Brake rate: ");
        move(4,0);
        printw("R axis: ");
        while(ros::ok()){
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1.0/60.0))
            );
            move(1, 20);
            printw("%3f", control_state.steer_angle);
            move(1, 30);
            printw("%d", axis[0]);
            move(2, 21);
            printw("%3f", control_state.effort);            
            move(2, 41);
            printw("%3f", control_state.maxBrakeRate);
            move(2, 30);
            printw("%d", axis[1]);
            if( num_of_axis > 2 )
            {
                move(3, 10);
                printw("%6d", axis[2] );
            }   
            if( num_of_axis > 3 )
            {
                move(4, 10);
                printw("%6d", axis[3] );
            }
            wrefresh(stdscr);
            
            std::this_thread::yield();
        }
    }
    
    /**
     * @brief: Background loop to read raw values from input steer
     * */
    void SteerLoop(){
        ssize_t r(0);
        while( ros::ok() ) 	/* infinite loop */
        {
            /* read the joystick state */
            r = read(joy_fd, &js, sizeof(struct js_event));   
            /* see what to do with the event */
            switch (js.type & ~JS_EVENT_INIT)
            {
                case JS_EVENT_AXIS:
                    axis   [ js.number ] = js.value;
                    break;
                case JS_EVENT_BUTTON:
                    button [ js.number ] = js.value;
                    break;
                default:
                    break;
            }
            std::this_thread::yield();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(szenergy::TELEOP_REFRESH_MS)
            );
        }
        close( joy_fd );
    }
};


#endif
