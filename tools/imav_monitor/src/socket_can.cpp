#include "socket_can.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCan::~SocketCan() { close(); }

bool SocketCan::open(const std::string& interfaceName, std::string& error) {
  close();
  lastError_.clear();

  descriptor_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (descriptor_ < 0) {
    error = std::string("socket(PF_CAN): ") + std::strerror(errno);
    lastError_ = error;
    return false;
  }

  const int acceptCanFd = 1;
  (void) ::setsockopt(descriptor_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                      &acceptCanFd, sizeof(acceptCanFd));

  ifreq interfaceRequest{};
  std::snprintf(interfaceRequest.ifr_name,
                sizeof(interfaceRequest.ifr_name), "%s",
                interfaceName.c_str());
  if (::ioctl(descriptor_, SIOCGIFINDEX, &interfaceRequest) < 0) {
    error = std::string("ioctl(SIOCGIFINDEX): ") + std::strerror(errno);
    lastError_ = error;
    close();
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interfaceRequest.ifr_ifindex;
  if (::bind(descriptor_, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) < 0) {
    error = std::string("bind(AF_CAN): ") + std::strerror(errno);
    lastError_ = error;
    close();
    return false;
  }

  return true;
}

void SocketCan::close() {
  if (descriptor_ >= 0) {
    ::close(descriptor_);
    descriptor_ = -1;
  }
}

bool SocketCan::isOpen() const { return descriptor_ >= 0; }

int SocketCan::receive(CanardCANFrame& output) {
  if (descriptor_ < 0) {
    lastError_ = "CAN socket is not open";
    return -1;
  }

  canfd_frame frame{};
  const ssize_t received = ::read(descriptor_, &frame, sizeof(frame));
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    lastError_ = std::string("read(SocketCAN): ") + std::strerror(errno);
    return -1;
  }

  if (received == static_cast<ssize_t>(sizeof(can_frame))) {
    const auto* classic = reinterpret_cast<const can_frame*>(&frame);
    output.id = classic->can_id;
    output.data_len = classic->can_dlc;
#if CANARD_ENABLE_CANFD
    output.canfd = false;
#endif
    output.iface_id = 0;
    std::memcpy(output.data, classic->data, output.data_len);
    return 1;
  }

  if (received == static_cast<ssize_t>(sizeof(canfd_frame))) {
    output.id = frame.can_id;
    output.data_len = frame.len;
#if CANARD_ENABLE_CANFD
    output.canfd = true;
#endif
    output.iface_id = 0;
    std::memcpy(output.data, frame.data, output.data_len);
    return 1;
  }

  return 0;
}

const std::string& SocketCan::lastError() const { return lastError_; }
