#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

extern "C" {
#include "mercuryapi.h"
}

using namespace std::chrono_literals;

class RfidTmreaderNode : public rclcpp::Node
{
public:
  RfidTmreaderNode()
  : Node("rfid_tmreader_node")
  {
    declare_parameter<std::string>("uri", "tmr:///dev/ttyUSB0");
    declare_parameter<int>("timeout_ms", 200);
    declare_parameter<int>("loop_sleep_ms", 100);
    declare_parameter<int>("max_tags", 64);
    declare_parameter<int>("epc_buf_size", 256);

    const auto uri = get_parameter("uri").as_string();
    timeout_ms_ = get_parameter("timeout_ms").as_int();
    loop_sleep_ms_ = get_parameter("loop_sleep_ms").as_int();
    max_tags_ = get_parameter("max_tags").as_int();
    epc_buf_size_ = get_parameter("epc_buf_size").as_int();

    pub_ = create_publisher<std_msgs::msg::String>("rfid/tags", 10);

    mercuryapi_result_t rc = mercuryapi_reader_create(&reader_);
    if (rc != MERCURYAPI_OK || !reader_) {
      throw std::runtime_error("mercuryapi_reader_create failed");
    }

    rc = mercuryapi_reader_connect(reader_, uri.c_str());
    if (rc != MERCURYAPI_OK) {
      mercuryapi_reader_destroy(reader_);
      reader_ = nullptr;
      throw std::runtime_error("mercuryapi_reader_connect failed");
    }

    rc = mercuryapi_reader_set_region_enum(reader_, MERCURYAPI_REGION_KR2);
    if (rc != MERCURYAPI_OK) {
      mercuryapi_reader_disconnect(reader_);
      mercuryapi_reader_destroy(reader_);
      reader_ = nullptr;
      throw std::runtime_error("mercuryapi_reader_set_region_enum failed");
    }

    ants_ = {1, 2};
    rc = mercuryapi_set_read_plan_gen2(reader_, ants_.data(), (int)ants_.size(), -1);
    if (rc != MERCURYAPI_OK) {
      mercuryapi_reader_disconnect(reader_);
      mercuryapi_reader_destroy(reader_);
      reader_ = nullptr;
      throw std::runtime_error("mercuryapi_set_read_plan_gen2 failed");
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(loop_sleep_ms_),
      std::bind(&RfidTmreaderNode::OnTimer, this));

    RCLCPP_INFO(get_logger(), "RFID node started. uri=%s", uri.c_str());
  }

  ~RfidTmreaderNode() override
  {
    if (reader_) {
      mercuryapi_reader_disconnect(reader_);
      mercuryapi_reader_destroy(reader_);
      reader_ = nullptr;
    }
  }

private:
  void OnTimer()
  {
    for (int ant : ants_) {
      mercuryapi_result_t rc = mercuryapi_set_read_plan_gen2(reader_, &ant, 1, -1);
      if (rc != MERCURYAPI_OK) {
        continue;
      }

      std::vector<std::vector<char>> storage(max_tags_, std::vector<char>(epc_buf_size_, 0));
      std::vector<char*> ptrs(max_tags_, nullptr);
      std::vector<int> lens(max_tags_, epc_buf_size_);

      for (int i = 0; i < max_tags_; ++i) {
        ptrs[i] = storage[i].data();
      }

      int count = 0;
      rc = mercuryapi_read_epcs(reader_, ptrs.data(), lens.data(),
                                max_tags_, &count, timeout_ms_);

      if (rc == MERCURYAPI_OK && count > 0) {
        std_msgs::msg::String msg;
        msg.data = "ant=" + std::to_string(ant) + " count=" + std::to_string(count);
        for (int i = 0; i < count; ++i) {
          msg.data += " ";
          msg.data += ptrs[i];
        }
        pub_->publish(msg);
      }
      // NO_TAG이면 조용히 무시
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mercuryapi_reader_t* reader_{nullptr};

  int timeout_ms_{200};
  int loop_sleep_ms_{100};
  int max_tags_{64};
  int epc_buf_size_{256};

  std::vector<int> ants_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RfidTmreaderNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
