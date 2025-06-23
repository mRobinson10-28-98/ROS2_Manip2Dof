#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "manip2dof_interfaces/msg/manip2_dof_solution.hpp"
#include "manip2dof_interfaces/srv/manip2_dof_properties.hpp"
#include "Manip2Dof.hpp"

using std::placeholders::_1;

static const double LINK1_LENGTH = 3;
static const double LINK2_LENGTH = 3;

class Manip2Dof_SolveGoal : public rclcpp::Node
{
  public:
    Manip2Dof_SolveGoal(Manip2Dof manip)
    : Node("Manip2Dof_SolveGoal"), mManip(manip)
    {
      mPosSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "Manip2Dof_SetGoalTraceCircle_PoseTopic", 10, std::bind(&Manip2Dof_SolveGoal::topic_callback, this, _1));

      mSolutionPub = this->create_publisher<manip2dof_interfaces::msg::Manip2DofSolution>("Manip2Dof_SolutionTopic", 10);

      mManipPropertiesService = this->create_service<manip2dof_interfaces::srv::Manip2DofProperties>(
        "request_manip_info", &Manip2Dof_SolveGoal::service_response);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose2D & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s','%s", std::to_string(msg.x).c_str(), std::to_string(msg.y).c_str());
      mManip.Ik(msg.x, msg.y);
      std::array<double, 2> s = mManip.GetJointConfiguration();

      auto sol = manip2dof_interfaces::msg::Manip2DofSolution();
      sol.j1 = s[0];
      sol.j2 = s[1];

      mSolutionPub->publish(sol);
    }

    static void service_response(const std::shared_ptr<manip2dof_interfaces::srv::Manip2DofProperties::Request> request,
      std::shared_ptr<manip2dof_interfaces::srv::Manip2DofProperties::Response> response)
    {
      response->l1 = LINK1_LENGTH;
      response->l2 = LINK2_LENGTH;
    } 

    // Subscriber object to get position goals 
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr mPosSub;

    // Inverse Kinematics solutions publisher
    rclcpp::Publisher<manip2dof_interfaces::msg::Manip2DofSolution>::SharedPtr mSolutionPub;

    // Service for clients to request kinematic properties of manipulator
    rclcpp::Service<manip2dof_interfaces::srv::Manip2DofProperties>::SharedPtr mManipPropertiesService;

    Manip2Dof mManip;
};

int main(int argc, char * argv[])
{
  Manip2Dof manip(LINK1_LENGTH, LINK2_LENGTH);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Manip2Dof_SolveGoal>(manip));
  rclcpp::shutdown();
  return 0;
}