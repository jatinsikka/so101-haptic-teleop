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

// Virtual fixture modes = the paper's three study conditions (Bloom et al., IEEE ToH 2026). Key [c] cycles.
enum class FixtureMode : int {
  kOff = 0,        // no fixture: nothing stops the arm (but the floor backstop)
  kImpedance = 1,  // eq. 9-10: spring on how far the HAND is past the wall; the arm is NOT stopped
  kCbf = 2,        // eq. 22-23: CBF filter stops the arm before the wall; force = D x speed removed
};
inline const char* mode_name(FixtureMode m) {
  switch (m) {
    case FixtureMode::kOff: return "OFF";
    case FixtureMode::kImpedance: return "IMPEDANCE";
    default: return "CBF";
  }
}

struct Config {
  bool enable_force = false;   // false = only zero force is ever sent
  WorkspaceMap map;            // scale + axis signs (anchors set at runtime). The hand drives the wrist point
  double max_joint_speed = 2.0;  // rad/s, rate limit on the arm target
  int stylus_axis = 0;         // column of the Touch rotation matrix along the stylus
  double pitch_gain = 1.0;     // stylus pitch -> tool pitch (0 = hold wrist)
  double roll_gain = 1.0;      // stylus twist -> wrist_roll
  double max_force_N = 0.8;
  double slew_N_per_s = 10.0;
  double engage_ramp_s = 0.5;
  double free_damping = 0.3;   // N/(m/s) while engaged
  double velocity_lpf_hz = 30.0;  // low-pass on the stylus velocity used by all damping

  // ---- virtual fixture (a plane on the tool tip)
  Wall wall;
  FixtureMode mode = FixtureMode::kImpedance;  // start mode
  bool filter_always = false;  // [f]: CBF filter on the arm in EVERY mode (the paper's user study)
  double floor_below = 0.015;  // m: hard floor this far below the wall (the table). The commanded tip never passes it
  double cbf_gain = 10.0;      // K [1/s]: the arm chases the hand at u = K x gap (eq. 7). Up/Down live
  double cbf_alpha = 3.33;     // alpha [1/s]: approach at most alpha x height (eq. 14). Left/Right live
  double cbf_damping = 40.0;   // D [N s/m at the stylus]: CBF force = D x speed removed (eq. 23)

  // ---- other forces
  bool tracking_enabled = true;      // [h]: eq. 8 "rubber band" from the hand to the robot
  double tracking_stiffness = 15.0;  // N/m at the stylus (the paper's Kp)
  bool reach_enabled = true;   // spring toward what the arm can reach
  double reach_stiffness = 100.0;

  double watchdog_s = 0.1;     // follower states older than this are ignored
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
  bool engaged = false, fault = false, follower_live = false, clamped = false;
  FixtureMode mode = FixtureMode::kImpedance;
  bool filter_on = false;      // CBF filter acting on the arm this tick (CBF mode, or [f])
  bool tracking_on = false;
  bool hand_in_wall = false;   // the hand is past the wall (impedance force zone)
  bool at_floor = false;       // the floor backstop is holding the arm
  double cbf_removed = 0;      // speed the filter removed toward the wall [m/s, arm side]; > 0 = active
  double cbf_gain = 0, cbf_alpha = 0;  // live K and alpha [1/s]
  double f_wall = 0, f_cbf = 0, f_track = 0;  // force components [N], before the safety filter
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
  // window keys. Mode/filter/tracking only while button 1 is released; K and alpha anytime.
  std::atomic<int> mode{static_cast<int>(FixtureMode::kImpedance)};  // [c]
  std::atomic<bool> filter_always{false};  // [f]
  std::atomic<bool> tracking_on{true};     // [h]
  std::atomic<double> cbf_gain{10.0};      // Up/Down
  std::atomic<double> cbf_alpha{3.33};     // Left/Right, [1]-[5] = K/alpha presets
};

}  // namespace touch
