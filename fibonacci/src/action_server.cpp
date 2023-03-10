#include <inttypes.h>
#include <memory>
#include "hello_world_msgs/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacchi = hello_world_msgs::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacchi>;

class MinimalActionServer : public rclcpp::Node
{
public:
   explicit MinimalActionServer()
   : Node("minimal_action_server")
   {
      using namespace std::placeholders;

      // fibonacciアクションのサーバ設定
      this->action_server_ = rclcpp_action::create_server<Fibonacchi>(
         this->get_node_base_interface(),
         this->get_node_clock_interface(),
         this->get_node_logging_interface(),
         this->get_node_waitables_interface(),
         "fibonacci",
         std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
         std::bind(&MinimalActionServer::handle_cancel, this, _1),
         std::bind(&MinimalActionServer::handle_accepted, this, _1));
   }

private:
   rclcpp_action::Server<Fibonacchi>::SharedPtr action_server_;

   // 目標値の設定時に呼び出されるハンドラ
   rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Fibonacchi::Goal> goal)
    {
       RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
       (void)uuid;
       // 9000以上は拒否
       if (goal->order > 9000) {
          return rclcpp_action::GoalResponse::REJECT;
       }
       return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // アクションのキャンセル時に呼び出されるハンドラ
    rclcpp_action::CancelResponse handle_cancel(
       const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
       RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
       (void)goal_handle;
       return rclcpp_action::CancelResponse::ACCEPT;
    }

    // アクション実行の中身
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
       RCLCPP_INFO(this->get_logger(), "Executing goal");
       rclcpp::Rate loop_rate(1);
       const auto goal = goal_handle->get_goal();
       auto feedback = std::make_shared<Fibonacchi::Feedback>();
       auto & sequence = feedback->sequence;
       sequence.push_back(0);
       sequence.push_back(1);
       auto result = std::make_shared<Fibonacchi::Result>();

       for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
          // アクションがキャンセルされていないか確認
          if (goal_handle->is_canceling()) {
             result->sequence = sequence;
             goal_handle->canceled(result);
             RCLCPP_INFO(this->get_logger(), "Goal canceled");
             return;
          }
          // sequenceの更新
          sequence.push_back(sequence[i] + sequence[i - 1]);
          // フィードバックの送信
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publish Feedback");

          loop_rate.sleep();
       }

       // 目標到達の確認
       if (rclcpp::ok()) {
          result->sequence = sequence;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
       }
    }

    // アクションの実行開始時に呼び出されるハンドラ
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
       using namespace std::placeholders;
       // executeメソッドの実行でExecutorの処理がブロックされないようにスレッド実行
       std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
    }
};

int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);
   auto action_server = std::make_shared<MinimalActionServer>();

   rclcpp::spin(action_server);
   rclcpp::shutdown();
   return 0;
}