#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "Manip2Dof.hpp"

using std::placeholders::_1;

class Manip2DofSubscriber : public rclcpp::Node
{
  public:
    Manip2DofSubscriber()
    : Node("manip2dof_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "manip2dof_pose_topic", 10, std::bind(&Manip2DofSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose2D & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s','%s", std::to_string(msg.x).c_str(), std::to_string(msg.y).c_str());
      mManip.Ik(msg.x, msg.y);
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;
    Manip2Dof mManip;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Manip2DofSubscriber>());
  rclcpp::shutdown();
  return 0;
}