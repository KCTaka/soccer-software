// Copyright 2026 UTRA-RoboSoccer
// stm32_link_probe <device> [window_ms]: see link_probe.hpp. Read-only.

#include <chrono>
#include <cstdlib>
#include <iostream>

#include "humanoid_transport_stm32/link_probe.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: stm32_link_probe <device, e.g. /dev/robosoccer-master> [window_ms]\n"
              << "Opens the master's serial port and listens. Sends nothing.\n";
    return 2;
  }
  const std::chrono::milliseconds window{argc == 3 ? std::atoi(argv[2]) : 2000};
  return humanoid::transport_stm32::probe_master(argv[1], window, std::cout) ? 0 : 1;
}
