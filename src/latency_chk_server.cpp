#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>
#include <absl/strings/str_format.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "latency_chk_proto/msg/latency_check_request.hpp"
#include "latency_chk_proto/msg/latency_check_payload.hpp"

class SystemHealthCheckServerImpl : public rclcpp::Node
{
public:
    SystemHealthCheckServerImpl()
    : Node("latency_check_server")
    {
        sub_chk_request_ = this->create_subscription<latency_chk_proto::msg::LatencyCheckRequest>(
            "latency_chk_request", 10, std::bind(&SystemHealthCheckServerImpl::onChkLatencyRequest, this, std::placeholders::_1));
        pub_chk_response_ = this->create_publisher<latency_chk_proto::msg::LatencyCheckPayload>(
            "latency_chk_response",
            //rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort()
            rclcpp::QoS(rclcpp::KeepLast(1000)).reliable()
        );
        pub_chk_complete_ = this->create_publisher<std_msgs::msg::Int32>(
            "latency_chk_complete", 10);
    }

private:
    void notifyLatencyCheckComplete(int size) {
        std_msgs::msg::Int32 msg;
        msg.data = size;
        pub_chk_complete_->publish(msg);
    }

    void do_run(const int runs, int snd_size) {
        // log parameter
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Runs                    : %d", runs);
        RCLCPP_INFO(this->get_logger(), "Message size            : %d KB",  snd_size / 1024);

        latency_chk_proto::msg::LatencyCheckPayload msg;

        // prepare send buffer
        msg.header.frame_id = "";
        std::vector<unsigned char> payload(snd_size + 1, 'a');
        msg.set__body(payload);

        // run test
        int run(0);
        for (; run < runs; ++run) {
            // header
#ifdef _WIN32
            FILETIME ft;
            GetSystemTimeAsFileTime(&ft);
            UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
            // A Windows tick is 100 nanoseconds. Windows epoch 1601-01-01T00:00:00Z
            // is 11644473600 seconds before Unix epoch 1970-01-01T00:00:00Z.
            msg.header.stamp.sec = ((INT64)((ticks / 10000000) - 11644473600LL));
            msg.header.stamp.nanosec = ts->set_nanos((INT32)((ticks % 10000000) * 100));
#else
            struct timeval tv;
            gettimeofday(&tv, NULL);
            msg.header.stamp.sec = (tv.tv_sec);
            msg.header.stamp.nanosec = (tv.tv_usec * 1000);
#endif
            std::string s = absl::StrFormat("s%d-r%d/%d", snd_size, (run+1), runs);
            msg.header.frame_id = s;
            pub_chk_response_->publish(msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // log test
        RCLCPP_INFO(this->get_logger(), "Messages sent           : %d", run);
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");

        // let the receiver do the evaluation
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // send check complete notification
        notifyLatencyCheckComplete(snd_size);
        RCLCPP_INFO(this->get_logger(), "Complete latency check !!!    ");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    }

    void onChkLatencyRequest(const latency_chk_proto::msg::LatencyCheckRequest& request) {

        int runs = request.runs;
        int size = request.size;

        RCLCPP_INFO(this->get_logger(), "Latency check requested.(size = %d, runs = %d)", size, runs);

        if (runs <= 0 || size <= 0) {
            RCLCPP_INFO(this->get_logger(), "Latency check canceled.");
            return;
        }
        
        do_run(runs, size);
    }

    rclcpp::Subscription<latency_chk_proto::msg::LatencyCheckRequest>::SharedPtr sub_chk_request_;
    rclcpp::Publisher<latency_chk_proto::msg::LatencyCheckPayload>::SharedPtr pub_chk_response_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_chk_complete_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemHealthCheckServerImpl>());
    rclcpp::shutdown();
    return 0;
}