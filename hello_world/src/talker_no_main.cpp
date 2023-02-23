#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name)
  : Node("talker")
  {
    // タイマーが実行されるイベントハンドラー関数
    auto publish_message =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {
        // 送信するメッセージ
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello world!";

        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        pub_->publish(std::move(msg));
      };
    
    // chatterトピックの送信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = create_publisher<std_msgs::msg::String>(topic_name, qos);
    // publish_messageの100ミリ秒周期でのタイマー実行
    timer_ = create_wall_timer(100ms, publish_message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};