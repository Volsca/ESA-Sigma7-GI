#include <chrono>
#include <memory>
#include "drdc.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#pragma region subclasses & enums
/**
 * @class S7StatePublisher
 * @brief ROS2 node that publishes the current state as a  (TODO)PoseStamped message.
 *
 * This class manages a publisher for the S7Mothership, taking care of publishing the state msgs.
 */
class S7StatePublisher : public rclcpp::Node
{
private:
    /**
     * @brief ROS2 publisher for (TODO)PoseStamped messages.
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    /**
     * @brief Current state stored as a shared pointer to (TODO)PoseStamped.
     */
    geometry_msgs::msg::PoseStamped::SharedPtr current_state_;

public:
    /**
     * @brief Constructor for S7StatePublisher node.
     *
     * Initializes the ROS2 node with the name "state_publisher"
     * and creates a publisher on the topic "controller_pose_stamped"
     * , which is subscribed to by a component of S7Spaceship.
     */
    S7StatePublisher() : Node("state_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("controller_pose_stamped", 1);
    }

    /**
     * @brief Getter for current_state_.
     */
    geometry_msgs::msg::PoseStamped::SharedPtr getCurrentState()
    {
        if (current_state_)
        {
            return current_state_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "current_state_ is null when getting");
            return current_state_;
        }
    }

    /**
     * @brief Setter for current_state_.
     */
    void setCurrentState(geometry_msgs::msg::PoseStamped::SharedPtr state) { current_state_ = state; }

    /**
     * @brief Publish the current state
     *
     * Publishes the current state (current_state_) to controller_pose_stamped if not null
     *
     * @return 1 if published without fail, 0 if current_state_ is null.
     */
    int8_t publish()
    {
        if (current_state_)
        {
            publisher_->publish(*current_state_);
            return 1;
        }
        else
        {
            return 0;
        }
    }
};

/**
 * @class S7ForceSubscriber
 * @brief ROS2 node that subscribes to controller_wrench_topic and recieves F.F. instructions from S7Spaceship
 *
 * This class manages a subscriber for the S7Mothership, taking care of storing the current wrench msg.
 */
class S7ForceSubscriber : public rclcpp::Node
{
private:
    /**
     * @brief ROS2 subscriber for WrenchStamped messages.
     */
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;

    /**
     * @brief latest recieved F.F. message from S7Spaceship
     */
    geometry_msgs::msg::WrenchStamped::SharedPtr current_wrench_;

public:
    /**
     * @brief Constructor for S7ForceSubscriber node.
     *
     * Initializes the ROS2 node with the name "wrench_subscriber"
     * and creates a subscriber on the topic "controller_wrench_stamped"
     * containing F.F. instructions for the S7 arm, which is published
     * to by a component of S7Spaceship.
     */
    S7ForceSubscriber() : Node("wrench_subscriber")
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "controller_wrench_stamped", 1,
            std::bind(&S7ForceSubscriber::messageCallback, this, std::placeholders::_1));
    }

    /**
     * @brief Getter for current_wrench_
     *
     * @returns a geometry_msgs::msg::WrenchStamped::SharedPtr
     */
    geometry_msgs::msg::WrenchStamped::SharedPtr getCurrentWrench()
    {
        if (current_wrench_)
        {
            return current_wrench_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "current_wrench_ null when getting");
            return current_wrench_;
        }
    }

    /**
     * @brief Setter for current_wrench_
     */
    void setCurrentWrench(geometry_msgs::msg::WrenchStamped::SharedPtr wrench) { current_wrench_ = wrench; }

private:
    /**
     * @brief Callback function for the subscriber, storing the message
     * in current_wrench_
     */
    void messageCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr message)
    {
        if (message)
        {
            setCurrentWrench(message);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "message in S7ForceSubscriber is null when setting current_wrench_");
            setCurrentWrench(message);
        }
    }
};

class S7InterfaceControlPublisher : public rclcpp::Node
{
public:
    S7InterfaceControlPublisher() : Node("state_publisher") {}
};

class S7InterfaceControlsubscriber : public rclcpp::Node
{
public:
    S7InterfaceControlsubscriber() : Node("state_publisher") {}
};

class S7Controller
{
};

class S7RingBuffer
{
};

enum S7Mode
{
    BRAKING,
    FREE,
    SELF_CENTERING
};
#pragma endregion

class S7Mothership : public rclcpp::Node
{
private:
    // member objects
    S7StatePublisher StatePublisher_;
    S7ForceSubscriber ForceSubscriber_;
    S7Controller Controller_;
    S7InterfaceControlPublisher ControlPublisher_;
    S7InterfaceControlsubscriber ControlSubscriber_;
    S7RingBuffer forceMessages_;
    S7RingBuffer stateMessages_;
    S7Mode CurrentMode_;
    S7Mode RequestedMode_;
};