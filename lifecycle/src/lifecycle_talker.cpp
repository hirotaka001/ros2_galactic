#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
// 状態遷移の返り値 CallbackReturn エイリアスの作成
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
   explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
   : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
   {}

   void publish()
   {
      static size_t count = 0;
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "Lifecycle Hello World #" + std::to_string(++count);

      if (!pub_->is_activated()) {
         RCLCPP_INFO(
            get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
      } else {
         RCLCPP_INFO(
            get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
      }

      pub_->publish(std::move(msg));
   }
   
   // 初期化時に呼び出されるコールバックメソッド
   CallbackReturn on_configure(const rclcpp_lifecycle::State &)
   {
      pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
      timer_ = this->create_wall_timer(
        1s, std::bind(&LifecycleTalker::publish, this));

      RCLCPP_INFO(get_logger(), "on_configure() is called.");

      return CallbackReturn::SUCCESS;
   }

   // スピン状態への移行時に呼び出されるコールバックメソッド
   CallbackReturn on_activate(const rclcpp_lifecycle::State &)
   {
      pub_->on_activate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

      std::this_thread::sleep_for(2s);

      return CallbackReturn::SUCCESS;
   }

   // スピン停止状態への移行時に呼び出されるコールバックメソッド
   CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
   {
      pub_->on_deactivate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

      return CallbackReturn::SUCCESS;
   }

   // 再初期化時に呼び出されるコールバックメソッド
   CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
   {
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

      return CallbackReturn::SUCCESS;
   }

   // 終了時に呼び出されるコールバックメソッド
   CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
   {
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(
         get_name(),
         "on shutdown is called from state %s.",
         state.label().c_str());

      return CallbackReturn::SUCCESS;
   }

private:
   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
   std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   rclcpp::init(argc, argv);
   rclcpp::executors::SingleThreadedExecutor exe;

   std::shared_ptr<LifecycleTalker> lc_node =
      std::make_shared<LifecycleTalker>("lc_talker");

   exe.add_node(lc_node->get_node_base_interface());
   exe.spin();

   rclcpp::shutdown();

   return 0;
}