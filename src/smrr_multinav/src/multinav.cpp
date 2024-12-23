#include "multinav.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir = 
        ament_index_cpp::get_package_share_directory("smrr_multinav") + "/bt_xml";

MultiNav::MultiNav(const std::string &node_name) : Node(node_name)
{   
    this->declare_parameter("location_file", "none");
    RCLCPP_INFO(get_logger(), "Init Done");
}

void MultiNav::setup()
{
    create_behavior_tree();

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MultiNav::update_behavior_tree, this)
    );

}

void MultiNav::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };

    factory.registerBuilder<GoToPose>("GoToPose", builder);
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
}

void MultiNav::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickRoot();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        timer_->cancel();
    }
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiNav>("multi_nav");
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();

}