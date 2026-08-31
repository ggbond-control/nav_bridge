/// @file udp_transport.cpp
/// @brief UDP通信封装实现

#include "nav_bridge/x30/udp_transport.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace nav_bridge {

UdpTransport::~UdpTransport() {
    close();
}

bool UdpTransport::open(const std::string &remote_ip, int remote_port, int local_port) {
    if (isOpen()) {
        close();
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }

    // 允许端口复用
    int opt = 1;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定本地端口
    if (local_port > 0) {
        struct sockaddr_in local_addr{};
        local_addr.sin_family      = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port        = htons(static_cast<uint16_t>(local_port));

        if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&local_addr),
                   sizeof(local_addr)) < 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
    }

    // 设置默认发送目标
    if (!remote_ip.empty() && remote_port > 0) {
        remote_addr_ = makeAddr(remote_ip, remote_port);
        has_remote_  = true;
    }

    return true;
}

void UdpTransport::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    has_remote_ = false;
}

bool UdpTransport::send(const void *data, size_t len) {
    if (!isOpen() || !has_remote_) return false;

    ssize_t sent =
        ::sendto(socket_fd_, data, len, 0, reinterpret_cast<const struct sockaddr *>(&remote_addr_),
                 sizeof(remote_addr_));
    return sent == static_cast<ssize_t>(len);
}

bool UdpTransport::sendTo(const void *data, size_t len, const std::string &ip, int port) {
    if (!isOpen()) return false;

    struct sockaddr_in target = makeAddr(ip, port);
    ssize_t sent              = ::sendto(socket_fd_, data, len, 0,
                                         reinterpret_cast<const struct sockaddr *>(&target), sizeof(target));
    return sent == static_cast<ssize_t>(len);
}

int UdpTransport::receive(void *buffer, size_t max_len, int timeout_ms) {
    if (!isOpen()) return -1;

    // 使用 poll 实现超时控制
    if (timeout_ms >= 0) {
        struct pollfd pfd;
        pfd.fd      = socket_fd_;
        pfd.events  = POLLIN;
        pfd.revents = 0;

        int ret = ::poll(&pfd, 1, timeout_ms);
        if (ret <= 0) {
            return ret;  // 0=超时, -1=错误
        }
    }

    struct sockaddr_in sender_addr{};
    socklen_t addr_len = sizeof(sender_addr);
    ssize_t received   = ::recvfrom(socket_fd_, buffer, max_len, 0,
                                    reinterpret_cast<struct sockaddr *>(&sender_addr), &addr_len);
    return static_cast<int>(received);
}

struct sockaddr_in UdpTransport::makeAddr(const std::string &ip, int port) {
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port));
    ::inet_aton(ip.c_str(), &addr.sin_addr);
    return addr;
}

}  // namespace nav_bridge
