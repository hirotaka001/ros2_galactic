#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

class LifecycleListener : public rclcpp::Node
{
public:
   explicit LifecycleListener(const std::string & node_name)
   : Node(node_name)
   {
      // lifecycle_chatterトピックの受信設定
      sub_data_ = this->create_subscription<std_msgs::msg::String>(
         "lifecycle_chatter", 10,
         std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));

      // lc_talkerノードの状態遷移トピックの受信設定
      sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
         "/lc_talker/transition_event",
         10,
         std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1));
   }

   // トピック受信時に呼び出されるメソッド
   void data_callback(const std_msgs::msg::String::SharedPtr msg)
   {
      RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
   }

   // ノードのライフサイクル変化をトピックとして扱い、受信時に呼び出されるメソッド
   void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
   {
      RCLCPP_INFO(
         get_logger(), "notify callback: Transition from state %s to %s",
         msg->start_state.label.c_str(), msg->goal_state.label.c_str());
   }

private:
   std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
   std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>>
      sub_notification_;
};

int main(int argc, char ** argv)
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   rclcpp::init(argc, argv);

   auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
   rclcpp::spin(lc_listener);

   rclcpp::shutdown();

   return 0;
}