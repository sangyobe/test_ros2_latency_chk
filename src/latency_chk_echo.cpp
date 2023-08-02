#include <absl/strings/str_format.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

#include "latency_chk_proto/msg/latency_check_payload.hpp"
#include "latency_chk_proto/msg/latency_check_request.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

namespace latency_chk 
{

class EchoServerImpl : public rclcpp::Node {
public:
  EchoServerImpl(const rclcpp::NodeOptions & options) 
  : Node("latency_check_echo", options) {
    sub_chk_request_ =
        this->create_subscription<latency_chk_proto::msg::LatencyCheckPayload>(
            "latency_chk_payload", 
            //rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&EchoServerImpl::onChkLatencyRequest, this,
                      std::placeholders::_1));
    pub_chk_response_ =
        this->create_publisher<latency_chk_proto::msg::LatencyCheckPayload>(
            "latency_chk_payload_echo",
            // rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile()
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    std::for_each(options.arguments().begin(), options.arguments().end(), [this](std::string arg) {
      RCLCPP_INFO(this->get_logger(), "arg: %s", arg.c_str());
    });
    std::for_each(options.parameter_overrides().begin(), options.parameter_overrides().end(), [this](rclcpp::Parameter param) {
      RCLCPP_INFO(this->get_logger(), "param: %s", param.value_to_string().c_str());
    });

    RCLCPP_INFO(this->get_logger(), "use_intra_process_comms: %s", (get_node_options().use_intra_process_comms() ? "True" : "False"));
  }

private:
  void
  onChkLatencyRequest(const latency_chk_proto::msg::LatencyCheckPayload &msg) {

    //RCLCPP_INFO(this->get_logger(), "Latency check message received.");

    pub_chk_response_->publish(msg);
  }

  rclcpp::Subscription<latency_chk_proto::msg::LatencyCheckPayload>::SharedPtr
      sub_chk_request_;
  rclcpp::Publisher<latency_chk_proto::msg::LatencyCheckPayload>::SharedPtr
      pub_chk_response_;
};

} // namespace latency_chk 

#ifdef BUILD_AS_COMPONENT

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(latency_chk::EchoServerImpl)

#else // BUILD_AS_COMPONENT

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::Node::SharedPtr node = std::make_shared<latency_chk::EchoServerImpl>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

#endif // BUILD_AS_COMPONENT