#include "net_ft_driver/bias_service.hpp"

#include <netinet/in.h>

#include <cstring>
#include <functional>
#include <string>

#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/cURLpp.hpp"

namespace
{
constexpr int kRdtPort = 49152;
constexpr uint16_t kRdtHeader = 0x1234;
constexpr uint16_t kCommandSize = 8;
constexpr uint32_t kBias = 0x0042;
constexpr int kNumGauges = 6;
}  // namespace

namespace net_ft_driver
{
BiasService::BiasService() : rclcpp::Node("net_ft_bias"), socket_(io_context_)
{
  ip_address_ = declare_parameter<std::string>("ip_address", "192.168.1.1");
  sensor_type_ = declare_parameter<std::string>("sensor_type", "ati_axia");

  socket_.open(asio::ip::udp::v4());
  socket_.connect(asio::ip::udp::endpoint(asio::ip::make_address(ip_address_), kRdtPort));

  set_bias_srv_ = create_service<Trigger>(
      "~/set_bias", std::bind(&BiasService::set_bias, this, std::placeholders::_1, std::placeholders::_2));
  clear_bias_srv_ = create_service<Trigger>(
      "~/clear_bias", std::bind(&BiasService::clear_bias, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Bias service ready for '%s' sensor at %s", sensor_type_.c_str(), ip_address_.c_str());
}

bool BiasService::send_rdt_command(uint32_t command, uint32_t sample_count, std::string& error)
{
  try {
    const uint16_t header = htons(kRdtHeader);
    const uint16_t cmd = htons(static_cast<uint16_t>(command));
    const uint32_t count = htonl(sample_count);

    uint8_t buffer[kCommandSize];
    std::memcpy(&buffer[0], &header, sizeof(header));
    std::memcpy(&buffer[2], &cmd, sizeof(cmd));
    std::memcpy(&buffer[4], &count, sizeof(count));

    socket_.send(asio::buffer(buffer, kCommandSize));
    return true;
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
}

bool BiasService::set_cgi_variable(const std::string& cgi_name, const std::string& var_name,
                                   const std::string& value, std::string& error)
{
  try {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;
    const std::string url{ "http://" + ip_address_ + "/" + cgi_name + "?" + var_name + "=" + value };
    request.setOpt(new curlpp::options::Url(url));
    request.setOpt(new curlpp::options::Timeout(2));
    request.perform();
    return true;
  } catch (const curlpp::RuntimeError& e) {
    error = e.what();
  } catch (const curlpp::LogicError& e) {
    error = e.what();
  }
  return false;
}

void BiasService::set_bias(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res)
{
  // OnRobot overloads the sample-count field as an on/off flag; ATI ignores it.
  const uint32_t sample_count = (sensor_type_ == "onrobot") ? 255u : 0u;

  std::string error;
  res->success = send_rdt_command(kBias, sample_count, error);
  res->message = res->success ? "Bias command sent; RDT 0x0042 is not acknowledged by the sensor"
                              : "Failed to send bias command: " + error;
  if (res->success) {
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void BiasService::clear_bias(const Trigger::Request::SharedPtr /*req*/, Trigger::Response::SharedPtr res)
{
  std::string error;

  if (sensor_type_ == "onrobot") {
    res->success = send_rdt_command(kBias, 0u, error);
    res->message = res->success ? "Bias cleared" : "Failed to clear bias: " + error;
  } else {
    res->success = true;
    for (int i = 0; i < kNumGauges; ++i) {
      const std::string var = "setbias" + std::to_string(i);
      if (!set_cgi_variable("setting.cgi", var, "0", error)) {
        res->success = false;
        res->message = "Failed to clear " + var + ": " + error;
        break;
      }
    }
    if (res->success) {
      res->message = "Software bias values cleared";
    }
  }

  if (res->success) {
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}
}  // namespace net_ft_driver
