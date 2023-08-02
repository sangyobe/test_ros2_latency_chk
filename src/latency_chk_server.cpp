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

namespace latency_chk 
{
class SystemHealthCheckServerImpl : public rclcpp::Node
{
public:
    SystemHealthCheckServerImpl(const rclcpp::NodeOptions & options)
    : Node("latency_check_server", options)
    {
        sub_chk_request_ = this->create_subscription<latency_chk_proto::msg::LatencyCheckRequest>(
            "latency_chk_request", 
            rclcpp::QoS(rclcpp::KeepLast(10)), 
            std::bind(&SystemHealthCheckServerImpl::onChkLatencyRequest, this, std::placeholders::_1));
        pub_chk_response_ =
            this->create_publisher<latency_chk_proto::msg::LatencyCheckPayload>(
                "latency_chk_payload",
                //rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile());
                rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
        pub_chk_complete_ = this->create_publisher<std_msgs::msg::Int32>(
            "latency_chk_complete",
            rclcpp::QoS(rclcpp::KeepLast(10)));
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
        RCLCPP_INFO(this->get_logger(), "Message size            : %d KB", snd_size / 1024);

        latency_chk_proto::msg::LatencyCheckPayload msg;

        // prepare send buffer
        msg.header.frame_id = "";
        std::vector<unsigned char> payload(snd_size, 'a');
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

        // let the client receive the last LatencyCheckPayload message
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // send check complete notification
        notifyLatencyCheckComplete(snd_size);
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

} // namespace latency_chk 

#ifdef BUILD_AS_COMPONENT

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(latency_chk::SystemHealthCheckServerImpl)

#else // BUILD_AS_COMPONENT

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<latency_chk::SystemHealthCheckServerImpl>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

#endif // BUILD_AS_COMPONENT