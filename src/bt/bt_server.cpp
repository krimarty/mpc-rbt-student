#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options)
        : TreeExecutionServer(options) {}

    void onTreeCreated(BT::Tree& tree) override {
        logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    }

    std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                        bool /*was_cancelled*/) override {
        logger_cout_.reset();
        return std::nullopt;
    }

private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto server = std::make_shared<BTServer>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
