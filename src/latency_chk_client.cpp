#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tclap/CmdLine.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.h"
#include "latency_chk_proto/msg/latency_check_request.h"
#include "latency_chk_proto/msg/latency_check_payload.h"

class SystemHealthCheckClientImpl : public rclcpp::Node
{
public:
    SystemHealthCheckClientImpl(int runs, int size,
                  std::string &log_file)
    : Node("latency_check_client"), runs_(runs), size_(size), log_file_(log_file)
    {
        pub_chk_request_ = this->create_publisher<latency_chk_proto::msg::LatencyCheckRequest>(
            "latency_chk_request", 10);
        );
        sub_chk_response_ = this->create_subscription<latency_chk_proto::msg::LatencyCheckPayload>(
            "latency_chk_response", 10, std::bind(&SystemHealthCheckClientImpl::onChkLatencyResponse, this, std::placeholders::_1));
        );

        reqChkLatency(runs_, size_);

    }

private:
    void onChkLatencyResponse(const latency_chk_proto::msg::LatencyCheckPayload& response) const {
        RCLCPP_INFO(this->get_logger(), "latency_chk_response");
    }

    void reqChkLatency(int runs, int size) {

        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Runs                    : %d", runs);
        RCLCPP_INFO(this->get_logger(), "Message size            : %d kB", (size / 1024));

        latency_chk_proto::msg::LatencyCheckRequest request;
        //request.header...;
        pub_chk_request_.publish(request);
    }

private:
    rclcpp::Subscription<latency_chk_proto::msg::LatencyCheckPayload>::SharedPtr sub_chk_response_;
    rclcpp::Publisher<latency_chk_proto::msg::LatencyCheckRequest>::SharedPtr pub_chk_request_;
    int runs_;
    int size_;
    std::string log_file_
};

int main(int argc, char* argv[])
{
    try {
        // parse command line
        TCLAP::CmdLine cmd("latency_snd");
        TCLAP::ValueArg<int> runs("r", "runs", "Number of messages to send.", false,
                                1, "int");
        TCLAP::ValueArg<int> size("s", "size", "Messages size in kB.", false, 4096,
                                "int");
        TCLAP::ValueArg<std::string> log_file("l", "log_file",
                                            "Base file name to export results.",
                                            false, "", "string");
        cmd.add(runs);
        cmd.add(size);
        cmd.add(log_file);
        cmd.parse(argc, argv);

        // request test

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<SystemHealthCheckClientImpl>(
            runs.getValue(), size.getValue() * 1024, log_file.getValue()));
        rclcpp::shutdown();
    } catch (TCLAP::ArgException &e) // catch any exceptions
    {
        RCLCPP_ERROR(this->get_logger(), "error: %d for arg %s", e.error(), e.argId());
        return EXIT_FAILURE;
    }

    return 0;
}