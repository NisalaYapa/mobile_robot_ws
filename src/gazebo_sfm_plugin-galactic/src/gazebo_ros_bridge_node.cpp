#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_ros/node.hpp>

class GazeboRos2Bridge : public rclcpp::Node {
public:
  GazeboRos2Bridge()
    : Node("gazebo_ros2_bridge") {

    this->declare_parameter<std::string>("gazebo_topic", "model_states");
    this->pose_topic = this->get_parameter("gazebo_topic").as_string();

    // Initialize ROS 2 publisher
    this->ros2_publisher = this->create_publisher<geometry_msgs::msg::Pose>("actor_pose", 10);

    // Initialize Gazebo subscriber
    auto gazebo_node = std::make_shared<gazebo_ros::Node>(this);
    this->gazebo_subscriber = gazebo_node->create_subscription<gazebo_msgs::msg::ModelStates>(
      this->pose_topic,
      10,
      [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        this->OnGazeboPoseMsg(msg);
      }
    );
  }

private:
  void OnGazeboPoseMsg(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    // Example for getting position of the first model in the list
    if (!msg->name.empty()) {
      auto ros2_msg = geometry_msgs::msg::Pose();
      ros2_msg.position = msg->pose[0].position;
      ros2_msg.orientation = msg->pose[0].orientation;
      this->ros2_publisher->publish(ros2_msg);
    }
  }

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gazebo_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ros2_publisher;
  std::string pose_topic;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboRos2Bridge>());
  rclcpp::shutdown();
  return 0;
}
