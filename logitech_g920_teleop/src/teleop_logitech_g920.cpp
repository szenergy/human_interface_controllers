#include <logitech_g920_teleop/szenergy_logitech_teleop.hpp>

/**
 * Stand-alone application that reads from steering input
 * to control simulated or real vehicle
 * */
int main (int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "szelectricity_logitech_teleop");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    ros::AsyncSpinner spinner(4);
    // Define the device location of the steer input
    std::string input_dev("/dev/input/js0");
    // Define the steering parameters according to szelectricity
    szenergy::TeleopState state(
        -szenergy::szelectricity::MAX_STEER_DEG, 
         szenergy::szelectricity::MAX_STEER_DEG, 
         0.1, -0.1, 100.0f);
    // Initialize releoperation
    LogitechTeleop teleop(input_dev, state, nh);
    // Initialize ROS
    std::unique_ptr<RosTeleopPublisher> pub(new RosTeleopPublisher(nh));
    teleop.Init(std::move(pub));
    // On initialization failure, exit
    if (!teleop.InitSteer())
    {
        return -1;
    }
    spinner.start();
    ros::waitForShutdown();
    // Stop ncurses
    endwin();
    return 0;
    
}
