#include <memory>

#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class GazeboPose : public rclcpp::Node
{
  public:
    GazeboPose()
    : Node("gazebo_pose_node")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/platform_robot/odom_platform_robot", 
      10, std::bind(&GazeboPose::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg);
      std::cout << "I heard: " << std::endl;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPose>());
  rclcpp::shutdown();
  return 0;
}