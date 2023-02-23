#include <cinttypes>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "hello_world_msgs/srv/set_message.hpp"

using namespace std::chrono_literals;
using hello_world_msgs::srv::SetMessage;

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(const std::string & service_name)
  : Node("client_async")
  {
    client_ = create_client<SetMessage>(service_name);
    // サービスサーバーの起動待ち
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available.");
    }
    // リクエストの設定
    auto request = std::make_shared<SetMessage::Request>();
    request->message = "Hello service!";

    // 非同期処理
    {
        using ServiceResponseFuture =
          rclcpp::Client<SetMessage>::SharedFuture;
        auto response_received_callback = [this](
          ServiceResponseFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%s",
                            response->result ? "true" : "false");
            rclcpp::shutdown;
          };
        auto future_result = client_->async_send_request(
          request, response_received_callback);
    }

    //   同期処理
    //   {
    //     auto future_result = client_->async_send_request(request);
    //     if (rclcpp::spin_until_future_complete(
    //         this->shared_from_this(), future_result) ==
    //         rclcpp::executor::FutureReturnCode::SUCCESS) {
    //             RCLCPP_INFO(this->get_logger(), "%s",
    //                 future_result.get()->result ? "true" : "false");
    //             rclcpp::shutdown();
    //         }
    //   }
  }

private:
  rclcpp::Client<SetMessage>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ClientNode>("set_message");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}