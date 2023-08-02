#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>
#include <tclap/CmdLine.h>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "latency_chk_proto/msg/latency_check_request.hpp"
#include "latency_chk_proto/msg/latency_check_payload.hpp"

// evaluation
void evaluate(std::vector<long long> &lat_arr_, size_t rec_size_,
              size_t warmups_, std::string &log_file_) {
  std::stringstream ss;

  // remove warmup runs
  if (lat_arr_.size() >= warmups_) {
    lat_arr_.erase(lat_arr_.begin(), lat_arr_.begin() + warmups_);
  }

  // evaluate all
  const long long sum_msg = lat_arr_.size();
  ss << "--------------------------------------------" << std::endl;
  ss << "Messages received             : " << sum_msg << std::endl;
  if (sum_msg > (long long)warmups_) {
    const long long sum_time =
        std::accumulate(lat_arr_.begin(), lat_arr_.end(), 0LL);
    const long long avg_time = sum_time / sum_msg;
    auto min_it = std::min_element(lat_arr_.begin(), lat_arr_.end());
    auto max_it = std::max_element(lat_arr_.begin(), lat_arr_.end());
    const size_t min_pos = min_it - lat_arr_.begin();
    const size_t max_pos = max_it - lat_arr_.begin();
    const long long min_time = *min_it;
    const long long max_time = *max_it;
    ss << "Message size received         : " << rec_size_ / 1024 << " kB"
       << std::endl;
    ss << "Message average latency       : " << avg_time << " us" << std::endl;
    ss << "Message min latency           : " << min_time << " us @ " << min_pos
       << std::endl;
    ss << "Message max latency           : " << max_time << " us @ " << max_pos
       << std::endl;
    ss << "Throughput                    : "
       << static_cast<int>(((rec_size_ * sum_msg) / 1024.0 / 1024.0) /
                           (sum_time / 1000.0 / 1000.0))
       << " MB/s" << std::endl;
    ss << "                              : "
       << static_cast<int>(((rec_size_ * sum_msg) / 1024.0 / 1024.0 / 1024.0) /
                           (sum_time / 1000.0 / 1000.0))
       << " GB/s" << std::endl;
    ss << "                              : "
       << static_cast<int>((sum_msg / 1000.0) / (sum_time / 1000.0 / 1000.0))
       << " kMsg/s" << std::endl;
  }
  ss << "--------------------------------------------" << std::endl;

  // log to console
  std::cout << ss.str();

  // log into logfile (append)
  if (!log_file_.empty()) {
    std::ofstream ostream;
    ostream.open(log_file_, std::ios::out | std::ios::app);
    ostream << ss.str();
  }
}

void log2file(std::vector<long long> &lat_arr_, size_t rec_size_,
              std::string &log_file_) {
  if (!log_file_.empty()) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << rec_size_ / 1024;
    const std::string rec_size_s = ss.str();

    std::ofstream ostream(rec_size_s + "-" + log_file_);
    const std::ostream_iterator<long long> output_iterator(ostream, "\n");
    std::copy(lat_arr_.begin(), lat_arr_.end(), output_iterator);
  }
}

namespace latency_chk
{
class SystemHealthCheckClientImpl : public rclcpp::Node
{
public:
    SystemHealthCheckClientImpl(const rclcpp::NodeOptions & options)
    : Node("latency_check_client", options), size_running_(0), recv_running_(0)
    {
        runs_ = 1;
        size_ = -1;
        log_file_ = "";
        for(const rclcpp::Parameter& param : options.parameter_overrides()) {
            if (param.get_name() == "runs" && 
                param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    int runs = param.get_value<int>();
                    if ( runs > 0) runs_ = runs;
            }
            else if (param.get_name() == "size" && 
                param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    int size = param.get_value<int>();
                    if ( size > 0) size_ = size;
            }
            else if (param.get_name() == "log_file" && 
                param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    log_file_ = param.get_value<std::string>();
            }
        }


        pub_chk_request_ = this->create_publisher<latency_chk_proto::msg::LatencyCheckRequest>(
            "latency_chk_request", 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        sub_chk_response_ = this->create_subscription<
            latency_chk_proto::msg::LatencyCheckPayload>(
            "latency_chk_payload",
            //rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile(),
            rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
            std::bind(&SystemHealthCheckClientImpl::onChkLatencyResponse, this,
                      std::placeholders::_1));
        sub_chk_complete_ = this->create_subscription<std_msgs::msg::Int32>(
            "latency_chk_complete", 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
            std::bind(&SystemHealthCheckClientImpl::onChkLatencyComplete, this, std::placeholders::_1));

        timer_start_check_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&SystemHealthCheckClientImpl::StartCheck, this));
    }

private:
    void StartCheck() {
        RCLCPP_INFO(this->get_logger(), "======== StartCheck ========");
        checkLatencyIfNecessary();
        timer_start_check_.reset();
    }

    void checkLatencyIfNecessary() {

        if (size_ < 0) {
            if (size_running_ <= 0)
                size_running_ = 1 * 1024; // 1kB
            else if (size_running_ < 32768 * 1024) // 32 MB
                size_running_ *= 2;
            else
                return;

            recv_running_ = 0;
            reqChkLatency(runs_, size_running_, log_file_);

        } else if (size_running_ <= 0) {
            size_running_ = size_;
            latency_vector_.clear();
            recv_running_ = 0;
            reqChkLatency(runs_, size_running_, log_file_);
        }
    }

    void onChkLatencyComplete(const std_msgs::msg::Int32& msg) {
        if (msg.data == size_running_) {

            evaluate(latency_vector_, size_running_, 0, log_file_);
            log2file(latency_vector_, size_running_, log_file_);

            RCLCPP_INFO(this->get_logger(), "CheckLatency succeeded.");

            checkLatencyIfNecessary();
        }
        else {
            RCLCPP_ERROR(
                this->get_logger(), 
                "onChkLatencyComplete, size mismatch (size processing=%d, got=%d)",
                size_running_, 
                msg.data);
        }
    }

    void onChkLatencyResponse(const latency_chk_proto::msg::LatencyCheckPayload& res) {

        //RCLCPP_INFO(this->get_logger(), "latency_chk_response");

        long long latency_us;

#ifdef _WIN32
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
        // A Windows tick is 100 nanoseconds. Windows epoch 1601-01-01T00:00:00Z
        // is 11644473600 seconds before Unix epoch 1970-01-01T00:00:00Z.
        INT64 sec = (INT64)((ticks / 10000000) - 11644473600LL);
        INT32 usec = (INT32)((ticks % 10000000) * 100);
        latency_us = sec * 1000000 + usec;
#else
        struct timeval tv;
        gettimeofday(&tv, NULL);
        latency_us = tv.tv_sec * 1000000 + tv.tv_usec;
#endif
        latency_us -= ((long long)res.header.stamp.sec * 1000000 +
                     (long long)res.header.stamp.nanosec / 1000);
        latency_vector_.push_back(latency_us);

        recv_running_++;
    }

    void reqChkLatency(int runs, int size, std::string &log_file) {

        RCLCPP_INFO(this->get_logger(), "======== CheckLatency ========");
        RCLCPP_INFO(this->get_logger(), "runs: %d", runs);
        RCLCPP_INFO(this->get_logger(), "size: %d kB", (size / 1024));
        RCLCPP_INFO(this->get_logger(), "log_file: %s", log_file.c_str());
        RCLCPP_INFO(this->get_logger(), "------------------------------");

        latency_chk_proto::msg::LatencyCheckRequest request;
#ifdef _WIN32
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        UINT64 ticks = (((UINT64)ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
        // A Windows tick is 100 nanoseconds. Windows epoch 1601-01-01T00:00:00Z
        // is 11644473600 seconds before Unix epoch 1970-01-01T00:00:00Z.
        request.header.stamp.sec = ((INT64)((ticks / 10000000) - 11644473600LL));
        request.header.stamp.nanosec = ts->set_nanos((INT32)((ticks % 10000000) * 100));
#else
        struct timeval tv;
        gettimeofday(&tv, NULL);
        request.header.stamp.sec = (tv.tv_sec);
        request.header.stamp.nanosec = (tv.tv_usec * 1000);
#endif
        request.header.frame_id = "";
        request.runs = runs;
        request.size = size;

        //clear statistics
        latency_vector_.clear();
        recv_running_ = 0;
        size_running_ = size;

        //request.header...;
        pub_chk_request_->publish(request);
    }

private:
    rclcpp::Subscription<latency_chk_proto::msg::LatencyCheckPayload>::SharedPtr sub_chk_response_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_chk_complete_;
    rclcpp::Publisher<latency_chk_proto::msg::LatencyCheckRequest>::SharedPtr pub_chk_request_;
    rclcpp::TimerBase::SharedPtr timer_start_check_;
    int runs_;
    int size_;
    int size_running_;
    int recv_running_;
    std::string log_file_;
    std::vector<long long> latency_vector_;
};

} // namespace latency_chk


#ifdef BUILD_AS_COMPONENT

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(latency_chk::SystemHealthCheckClientImpl)

#else // BUILD_AS_COMPONENT

int main(int argc, char* argv[])
{
    try {
        // parse command line
        TCLAP::CmdLine cmd("latency_snd");
        TCLAP::ValueArg<int> runs("r", "runs", "Number of messages to send.", false,
                                1, "int");
        TCLAP::ValueArg<int> size_kb("s", "size", "Messages size in kB.", false, 1,
                                "int");
        TCLAP::ValueArg<std::string> log_file("l", "log_file",
                                            "Base file name to export results.",
                                            false, "", "string");
        cmd.add(runs);
        cmd.add(size_kb);
        cmd.add(log_file);
        cmd.parse(argc, argv);

        // request test

        rclcpp::init(argc, argv);
        rclcpp::executors::MultiThreadedExecutor exec;
        rclcpp::NodeOptions options;
        options.append_parameter_override<int>("runs", runs.getValue())
               .append_parameter_override<int>("size", size_kb.getValue() * 1024)
               .append_parameter_override<std::string>("log_file", log_file.getValue());
        rclcpp::Node::SharedPtr node = std::make_shared<latency_chk::SystemHealthCheckClientImpl>(options);
        exec.add_node(node);
        exec.spin();
        rclcpp::shutdown();
    } catch (TCLAP::ArgException &e) // catch any exceptions
    {
        //RCLCPP_ERROR(this->get_logger(), "error: %d for arg %s", e.error(), e.argId());
        return EXIT_FAILURE;
    }

    return 0;
}

#endif // BUILD_AS_COMPONENT
