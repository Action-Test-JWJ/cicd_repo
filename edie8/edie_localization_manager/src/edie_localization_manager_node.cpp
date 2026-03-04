#include "edie_localization_manager/edie_localization_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieLocalizationManager>();
    rclcpp::WallRate loop_rate(node->GetLoopRate());
    RCLCPP_INFO(node->get_logger(), "Edie Localization Manager started");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->main();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
