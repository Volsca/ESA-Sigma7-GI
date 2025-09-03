#include <chrono>
#include <memory>
#include "drdc.h"
#include "rclcpp/rclcpp.hpp"

// pre-define classes for naming
class S7StatePublisher;
class S7ForceSubscriber;
class S7InterfaceMsgPublisher;
class S7InterfaceInstructionSubscriber;

class S7Mothership : public rclcpp::Node
{
    private:
    std::shared_ptr<S7StatePublisher> _StatePublisher;

};

// #########################################
// ## Motherships' sub classes

class S7StatePublisher : public rclcpp::Node
{

};

class S7ForceSubscriber : public rclcpp::Node
{
    
};

class S7InterfaceMsgPublisher: public rclcpp::Node
{
    
};

class S7InterfaceInstructionSubscriber : public rclcpp::Node
{
    
};