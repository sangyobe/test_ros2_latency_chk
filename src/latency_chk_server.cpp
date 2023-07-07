#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.h"

class SystemHealthCheckImpl : public rclcpp::Node
{
public:
    SystemHealthCheckImpl()
    : Node("minimal_publisher")
    {
        sub_chk_request_ = this->create_subscription<std_msgs::msg::String>(
            "topic_name", 10, std::bind(&SystemHealthCheckImpl::onChkLatency, this, std::placeholders::_1));
        );
        pub_chk_response_ = this->create_publisher<std_msgs::msg::String>(
            "topic_name", 10);
        );
    }

private:
    void do_run(const int runs, int snd_size /*bytes*/,
         ::grpc::ServerWriter<::art_protocol::std_msgs::LatencyCheckPayload>
             *writer) {
        // log parameter
        LOG(INFO) << "--------------------------------------------";
        LOG(INFO) << "Runs                    : " << runs;
        LOG(INFO) << "Message size            : " << (snd_size / 1024) << " kB";

        // message LatencyCheckPayload {
        //   Header header = 1;
        //   bytes body = 2;
        // }
        art_protocol::std_msgs::LatencyCheckPayload msg;
        // prepare send buffer
        std::unique_ptr<char[]> payload = std::make_unique<char[]>(snd_size + 1);
        for (int i = 0; i < snd_size; i++)
            payload[i] = ('a' + (i % 26));
        payload[snd_size] = 0;

        // msg.set_body(std::move(std::string(payload)));
        msg.set_body(payload.get());

        // let them match
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // run test
        int run(0);
        for (; run < runs; ++run) {
            // header
            msg.mutable_header()->set_seq(run + 1);
        #ifdef _WIN32
            FILETIME ft;
            GetSystemTimeAsFileTime(&ft);
            UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
            // A Windows tick is 100 nanoseconds. Windows epoch 1601-01-01T00:00:00Z
            // is 11644473600 seconds before Unix epoch 1970-01-01T00:00:00Z.
            timestamp.set_seconds((INT64)((ticks / 10000000) - 11644473600LL));
            timestamp.set_nanos((INT32)((ticks % 10000000) * 100));
        #else
            struct timeval tv;
            gettimeofday(&tv, NULL);
            msg.mutable_header()->mutable_time_stamp()->set_seconds(tv.tv_sec);
            msg.mutable_header()->mutable_time_stamp()->set_nanos(tv.tv_usec * 1000);
        #endif
            msg.mutable_header()->set_frame_id("");

            writer->Write(msg);
        }

        // log test
        RCLCPP_INFO(this->get_logger(), "Messages sent           : ");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");

        // let the receiver do the evaluation
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    void onChkLatency(const std_msgs::msg::String& req) const {
        RCLCPP_INFO(this->get_logger(), "Requested. %s", req.data.c_str());

        int runs = request->runs();
    int size = request->size();

    if (runs <= 0 || size <= 0)
      return ::grpc::Status::CANCELLED;

    do_run(runs, size, writer);

    
        // auto message = std_msgs::msg::String();
        // message.data = "hello";
        // pub_chk_response_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_chk_request_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_chk_response_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemHealthCheckImpl>());
    rclcpp::shutdown();
    return 0;
}