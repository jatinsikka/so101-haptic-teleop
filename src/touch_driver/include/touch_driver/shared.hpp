// Config + data shared by the three threads:
//   haptic   (~1 kHz, CHAI3D): reads Touch, computes target + force, commands Touch
//   graphics (main, ~60 Hz):   draws the window
//   ROS      (spin):           publishes target, receives follower states
// Haptic thread only try_lock()s; on a miss it reuses last tick's copy. It never waits.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "touch_driver/haptic_math.hpp"
#include "touch_driver/kinematics.hpp"

namespace touch {

using Clock = std::chrono::steady_clock;

struct Config {
  bool enable_force = false;   // false = only zero force is ever sent
  WorkspaceMap map;            // scale + axis signs (anchors set at runtime)
  double max_joint_speed = 2.0;  // rad/s, rate limit on the arm target
  int stylus_axis = 0;         // column of the Touch rotation matrix along the stylus
  double pitch_gain = 1.0;     // stylus pitch -> tool pitch (0 = hold wrist)
  double roll_gain = 1.0;      // stylus twist -> wrist_roll
  double max_force_N = 0.8;
  double slew_N_per_s = 10.0;
  double engage_ramp_s = 0.5;
  double free_damping = 0.3;   // N/(m/s) while engaged
  double velocity_lpf_hz = 30.0;  // low-pass on the stylus velocity used by all damping
  bool wall_enabled = true;    // drawn + stops the arm; force only with enable_force
  Wall wall;
  bool reach_enabled = true;   // spring toward what the arm can reach
  double reach_stiffness = 100.0;
  bool feedback_enabled = false;  // spring toward the real follower tip
  double feedback_stiffness = 50.0;
  double feedback_deadband_m = 0.01;
  double feedback_lpf_hz = 15.0;
  double watchdog_s = 0.1;     // older follower states are ignored
  Arm5 home{0.0, -0.8, 0.8, 0.0, 0.0};  // ghost pose until the follower is heard
  std::string description_dir;          // so101_description share dir (meshes for the window)
};

enum class Fault : int { kNone = 0, kBadReading, kBadForce };
inline const char* fault_name(Fault f) {
  switch (f) {
    case Fault::kBadReading: return "non-finite reading from the Touch";
    case Fault::kBadForce: return "non-finite force computed";
    default: return "none";
  }
}

// ROS -> haptic
struct FromRos {
  Arm5 q_follower{};
  Clock::time_point stamp{};
  bool have_follower = false;
  bool clear_fault = false;
};

// haptic -> ROS + graphics (one tick's snapshot)
struct HapticState {
  std::uint64_t ticks = 0;
  double rate_hz = 0;
  bool engaged = false, fault = false, follower_live = false, in_wall = false, clamped = false;
  bool wall_on = false, feedback_on = false;
  Fault fault_reason = Fault::kNone;
  Vec3 force{};              // sent to the Touch (touch frame)
  double pitch = 0, twist = 0;
  Arm5 q_target{};           // ghost
  Arm5 q_follower{};
  Vec3 p_hand{}, p_reached{};  // arm frame, wrist points: where the hand maps, where the arm is sent
  Vec3 p_tool{};               // arm frame: tool tip
};

struct Shared {
  std::mutex from_ros_mtx, state_mtx;
  FromRos from_ros;
  HapticState state;
  std::atomic<bool> wall_on{false}, feedback_on{false};  // window keys (only when not engaged)
};

}  // namespace touch
