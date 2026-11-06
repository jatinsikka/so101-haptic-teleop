// CHAI3D side: Touch, 1 kHz haptic loop, 3D window. CHAI3D headers stay in haptics.cpp.
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "touch_driver/shared.hpp"

namespace touch {

class HapticApp {
 public:
  HapticApp(Shared& shared, const Config& cfg, const PlanarArm& arm, const RobotModel& model);
  ~HapticApp();  // zero force, stop loop, close device

  bool open(std::string& info);  // first CHAI3D device; false + reason if none
  // Starts the haptic thread, runs the window here until closed or keep_running() is false.
  // Returns after the haptic thread has stopped.
  void run(const std::function<bool()>& keep_running);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace touch
