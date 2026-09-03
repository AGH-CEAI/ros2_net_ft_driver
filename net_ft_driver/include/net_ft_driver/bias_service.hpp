#ifndef NET_FT_DRIVER__BIAS_SERVICE_HPP_
#define NET_FT_DRIVER__BIAS_SERVICE_HPP_

#include <cstdint>
#include <string>

#include "asio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace net_ft_driver
{
class BiasService : public rclcpp::Node
{
public:
  BiasService();
  ~BiasService() override = default;

private:
  using Trigger = std_srvs::srv::Trigger;

  void set_bias(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void clear_bias(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  bool send_rdt_command(uint32_t command, uint32_t sample_count, std::string& error);
  bool set_cgi_variable(const std::string& cgi_name, const std::string& var_name, const std::string& value,
                        std::string& error);

  std::string ip_address_;
  std::string sensor_type_;

  asio::io_context io_context_;
  asio::ip::udp::socket socket_;

  rclcpp::Service<Trigger>::SharedPtr set_bias_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_bias_srv_;
};
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__BIAS_SERVICE_HPP_
