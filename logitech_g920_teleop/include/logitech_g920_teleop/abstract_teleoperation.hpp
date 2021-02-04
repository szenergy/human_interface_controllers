#ifndef ABSTRACT_TELEOPERATION_HPP
#define ABSTRACT_TELEOPERATION_HPP

#include <ros/ros.h>

#include "common_usage.hpp"



class AbstractTeleopPublisher
{
public:
    virtual void Init() = 0;
    virtual void PublishCmd(const szenergy::RosTeleopPubState& teleop_pub_state) = 0;
};
/**
 * @brief: An ancestor class defining the ROS-related methods and fields
 * such as publishing/subscribing to a control topic
 * */
class RosTeleopPublisher: public AbstractTeleopPublisher
{
protected:
    // ROS components
    std::shared_ptr<ros::NodeHandle> nh; ///< ROS node handle
    ros::Publisher pub_cmd_torque; ///< Publish throttle
    ros::Publisher pub_cmd_steer; ///< Publish steer
    ros::Publisher pub_cmd_brake; ///< Publish brake
public:
    /**
     * @param<nh:RosNodeHandle>: ROS node handle
     * */
    RosTeleopPublisher(std::shared_ptr<ros::NodeHandle> nh): nh(nh)
    {
        
    }
    
    /**
     * @brief: Initialize ROS subscribers
     * */
    virtual void Init()
    {
        // Setup publishers
        pub_cmd_torque = nh->advertise<std_msgs::Float64>("/airsim_node/PhysXCar/car_cmd_body_frame/throttle", 
            szenergy::TELEOP_QUEUE_SIZE);
        pub_cmd_steer = nh->advertise<std_msgs::Float64>("/airsim_node/PhysXCar/car_cmd_body_frame/steering",
            szenergy::TELEOP_QUEUE_SIZE);
        pub_cmd_brake = nh->advertise<std_msgs::Float64>("/airsim_node/PhysXCar/car_cmd_body_frame/brake",
            szenergy::TELEOP_QUEUE_SIZE);
        
    }
    /**
     * @brief: Publish messages to the corresponding ROS topics
     * */
    virtual void PublishCmd(const szenergy::RosTeleopPubState& teleop_pub_state)
    {
        // Publish state
        pub_cmd_brake.publish(teleop_pub_state.msg_brake);
        pub_cmd_torque.publish(teleop_pub_state.msg_torque);
        pub_cmd_steer.publish(teleop_pub_state.msg_steer);
    }
};
/**
 * @brief: Abstract publisher for teleoperation stuff
 * 
 * */
class SzenergyRosTeleoperation
{
protected:    
    ros::Time timestamp; ///< Loaded timestamp
    std::shared_ptr<ros::NodeHandle> nh; ///< ROS node handle
    std::unique_ptr<AbstractTeleopPublisher> publisher;
    // State fields
    szenergy::RosTeleopPubState teleop_pub_state; ///< Store the message to be published
    szenergy::TeleopState control_state; ///< The teleoperation control state
    // ROS cmd_vel
    ros::Publisher pub_cmd_vel; ///< Publish geometry_twist
    ros::Publisher pub_autoware_cmd; ///< Publish autoware control command
    geometry_msgs::TwistStamped cmd_vel_msg;
    autoware_msgs::ControlCommandStamped cmd_autoware_msg;
public:
    SzenergyRosTeleoperation(const szenergy::TeleopState& state, std::shared_ptr<ros::NodeHandle> nh): control_state(state), nh(nh)
    {}
    
    /**
     * @brief: Initialize with publisher instance
     * */
    void Init(std::unique_ptr<AbstractTeleopPublisher> pub)
    {
        publisher = std::move(pub);
        publisher->Init();
        pub_cmd_vel = nh->advertise<geometry_msgs::TwistStamped>("/twist_cmd", szenergy::TELEOP_QUEUE_SIZE);
        pub_autoware_cmd = nh->advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 10);
        /// Initialize stamp
        cmd_vel_msg.header.frame_id = "base_link";
        cmd_autoware_msg.header.frame_id = "base_link";
    }

    /**
     * @brief: Publish through main connector
     * */
    void PublishCmd()
    {
        cmd_vel_msg.header.stamp = timestamp;
        cmd_autoware_msg.header.stamp = timestamp;
        publisher->PublishCmd(teleop_pub_state);
        pub_cmd_vel.publish(cmd_vel_msg);
        pub_autoware_cmd.publish(cmd_autoware_msg);
    }
    

    /**
     * @brief: Return with the read parameters
     * */
    szenergy::RosTeleopPubState& GetRosTeleopPubState()
    {
        return teleop_pub_state;
    }
};

#endif
