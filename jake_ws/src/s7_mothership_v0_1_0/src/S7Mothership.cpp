#include "s7_mothership_v0_1_0/S7Mothership.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mothership = std::make_shared<S7Mothership>();
    //auto testNode = rclcpp::Node("test_node");

    /*rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mothership->getNode());
    executor.spin();*/
    // rclcpp::spin(mothership->getNode());
    rclcpp::spin(mothership);//->getNode());
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}