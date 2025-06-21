#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Manip2Dof_SetGoalTraceCircle : public rclcpp::Node
{
  public:
    Manip2Dof_SetGoalTraceCircle()
    : Node("Manip2Dof_SetGoalTraceCircle"), count_(0)
    {
      mMsgPublisher_ = this->create_publisher<std_msgs::msg::String>("manip2dof_msg_topic", 10);
      mPosePublisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("Manip2Dof_SetGoalTraceCircle_PoseTopic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Manip2Dof_SetGoalTraceCircle::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);

      auto pose = geometry_msgs::msg::Pose2D();
      pose.theta = 0;
      iterate_circle(pose.x, pose.y);

      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      mMsgPublisher_->publish(message);
      mPosePublisher_->publish(pose);
    }

    void iterate_circle(double& x, double& y)
    {
      double theta = double(count_) / 10;

      x = circle_radius * cos(theta) + circle_x_offset;
      y = circle_radius * sin(theta) + circle_y_offset;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mMsgPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mPosePublisher_;
    size_t count_;

    double circle_x_offset {2};
    double circle_y_offset {2};
    double circle_radius {1};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Manip2Dof_SetGoalTraceCircle>());
  rclcpp::shutdown();
  return 0;
}