#pragma once

#include <canard.h>

#include <string>

/** Minimal non-blocking Linux SocketCAN reader adapted from MINICAN/TOOLS. */
class SocketCan {
public:
  ~SocketCan();

  bool open(const std::string& interfaceName, std::string& error);
  void close();
  bool isOpen() const;

  /** Return 1 for a frame, 0 when no frame is ready, and -1 on error. */
  int receive(CanardCANFrame& output);
  const std::string& lastError() const;

private:
  int descriptor_ = -1;
  std::string lastError_;
};
