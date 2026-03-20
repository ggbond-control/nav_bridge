/// @file nav_bridge_node.cpp
/// @brief Nav Bridge 节点入口

#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nav_bridge/x30_nav_bridge.hpp"

std::shared_ptr<nav_bridge::X30NavBridge> g_node = nullptr;

void signalHandler(int /*signum*/) {
    if (g_node) {
        g_node->shutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    g_node = std::make_shared<nav_bridge::X30NavBridge>(rclcpp::NodeOptions());

    if (!g_node->initialize()) {
        RCLCPP_FATAL(g_node->get_logger(), "Nav Bridge 初始化失败, 退出");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(g_node);

    g_node->shutdown();
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}
