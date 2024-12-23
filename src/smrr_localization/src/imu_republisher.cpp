#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

class ImuRepublisher : public rclcpp::Node
{
  public:
    ImuRepublisher()
    : Node("imu_republisher")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/out", 10, std::bind(&ImuRepublisher::topic_callback, this, _1));

      publisher_    = this->create_publisher<sensor_msgs::msg::Imu>("/imu/repub", 10);
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu & msg) const
    {
      //TODO: Complete publisher, neglecting z axis velocity, accelerations
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisher>());
  rclcpp::shutdown();
  return 0;
}
