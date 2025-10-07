#include "s7_mothership_v0_1_0/S7Mothership.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mothership = std::make_shared<S7Mothership>();
    mothership->init();
    rclcpp::spin(mothership);
    // mothership->~S7Mothership();
    rclcpp::shutdown();
    return 0;
}