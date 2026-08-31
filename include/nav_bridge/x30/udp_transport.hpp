#pragma once

/// @file udp_transport.hpp
/// @brief UDP通信封装 — 纯网络层, 不依赖ROS2

#include <netinet/in.h>

#include <string>

namespace nav_bridge {

/// @brief UDP socket 封装类
/// 支持发送到默认目标或指定目标, 非阻塞接收
class UdpTransport {
public:
    UdpTransport() = default;
    ~UdpTransport();

    // 不允许拷贝
    UdpTransport(const UdpTransport &)            = delete;
    UdpTransport &operator=(const UdpTransport &) = delete;

    /// 打开UDP连接
    /// @param remote_ip    默认发送目标IP
    /// @param remote_port  默认发送目标端口
    /// @param local_port   本地绑定端口 (0=系统分配)
    /// @return 成功返回true
    bool open(const std::string &remote_ip, int remote_port, int local_port = 0);

    /// 关闭连接
    void close();

    /// 发送数据到默认目标
    bool send(const void *data, size_t len);

    /// 发送数据到指定目标
    bool sendTo(const void *data, size_t len, const std::string &ip, int port);

    /// 非阻塞接收数据
    /// @param buffer    接收缓冲区
    /// @param max_len   缓冲区最大长度
    /// @param timeout_ms  超时毫秒数 (0=立即返回, -1=永久阻塞)
    /// @return 接收到的字节数, 0=超时, -1=错误
    int receive(void *buffer, size_t max_len, int timeout_ms = 0);

    /// 是否已打开
    bool isOpen() const { return socket_fd_ >= 0; }

    /// 获取socket fd (供高级用途)
    int fd() const { return socket_fd_; }

private:
    int socket_fd_ = -1;
    struct sockaddr_in remote_addr_{};
    bool has_remote_ = false;

    /// 将 ip:port 转换为 sockaddr_in
    static struct sockaddr_in makeAddr(const std::string &ip, int port);
};

}  // namespace nav_bridge
