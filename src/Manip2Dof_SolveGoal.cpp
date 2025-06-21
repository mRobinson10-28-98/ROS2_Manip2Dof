#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "Manip2Dof.hpp"

using std::placeholders::_1;

class Manip2Dof_SolveGoal : public rclcpp::Node
{
  public:
    Manip2Dof_SolveGoal()
    : Node("Manip2Dof_SolveGoal")
    {
      mPosSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "Manip2Dof_SetGoalTraceCircle_PoseTopic", 10, std::bind(&Manip2Dof_SolveGoal::topic_callback, this, _1));
      mSolutionPub = this->create_publisher<geometry_msgs::msg::Vector3>("Manip2Dof_SolutionTopic", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose2D & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s','%s", std::to_string(msg.x).c_str(), std::to_string(msg.y).c_str());
      mManip.Ik(msg.x, msg.y);
      std::array<double, 2> s = mManip.GetJointConfiguration();

      auto sol = geometry_msgs::msg::Vector3();
      sol.x = s[0];
      sol.y = s[1];

      mSolutionPub->publish(sol);
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr mPosSub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mSolutionPub;
    Manip2Dof mManip;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Manip2Dof_SolveGoal>());
  rclcpp::shutdown();
  return 0;
}