/**
 * @file healthSurvey.hpp
 * @brief Health survey publisher for node diagnostics.
 */
#pragma once
#include "UAVCAN/pubSub.hpp"

namespace HealthSurvey {
  /** @brief Start the periodic health survey publisher. */
  void start(UAVCAN::Node& _node);
}
