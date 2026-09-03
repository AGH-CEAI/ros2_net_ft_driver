#include <memory>

#include "net_ft_driver/bias_service.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<net_ft_driver::BiasService>());
  rclcpp::shutdown();
  return 0;
}
