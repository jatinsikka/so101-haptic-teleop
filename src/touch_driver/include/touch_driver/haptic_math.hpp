// Math + safety filters. No device/ROS/graphics code -> unit-tested in test/test_haptic_math.cpp.
//
// Frames:
//   touch: CHAI3D device frame. x toward you, y right, z up. [m]
//   arm:   SO-101 base_link.   x forward,     y left,  z up. [m]
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace touch {

using Vec3 = std::array<double, 3>;
using Arm5 = std::array<double, 5>;  // pan, lift, elbow, wrist_flex, wrist_roll

constexpr double kPi = 3.14159265358979323846;  // M_PI isn't standard C++

// Hard caps. Compiled in so the YAML can only go lower.
// Touch: 3.3 N peak, 0.88 N continuous, ~400 N/m, ~4 N/(m/s).
constexpr double kHardMaxForce_N = 1.5;
constexpr double kHardMaxStiffness_N_per_m = 250;
constexpr double kHardMaxDamping_Ns_per_m = 2.0;
constexpr double kHardMaxSlew_N_per_s = 50.0;

// ---- vectors
inline double clamp(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
inline Vec3 add(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 mul(double s, const Vec3& a) { return {s * a[0], s * a[1], s * a[2]}; }
inline double dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline bool finite(const Vec3& a) { return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]); }
inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }  // (-pi, pi]

// ---- 1. position map with clutch
// Anchors are set when button 1 is pressed:
//   p_arm = arm_anchor + scale * S * (p_touch - touch_anchor),   S = diag(sign)
// Release, re-center the stylus, press again: arm stays put (indexing).
// S is its own inverse and orthogonal, so the same S maps forces back.
struct WorkspaceMap {
  double scale = 2.0;
  Vec3 sign{1, -1, 1};  // toward you (+x) -> arm out (+x); your right (+y) -> arm's right (-y); up -> up
  Vec3 touch_anchor{};
  Vec3 arm_anchor{};
};

inline Vec3 flip(const Vec3& s, const Vec3& v) { return {s[0] * v[0], s[1] * v[1], s[2] * v[2]}; }

inline Vec3 touch_to_arm(const WorkspaceMap& m, const Vec3& p_touch) {
  return add(m.arm_anchor, mul(m.scale, flip(m.sign, sub(p_touch, m.touch_anchor))));
}
// Arm-frame displacement -> the stylus displacement that causes it. Springs use this,
// so every stiffness is "as felt at the stylus".
inline Vec3 arm_delta_to_touch(const WorkspaceMap& m, const Vec3& d_arm) { return mul(1.0 / m.scale, flip(m.sign, d_arm)); }
inline Vec3 arm_dir_to_touch(const WorkspaceMap& m, const Vec3& n_arm) { return flip(m.sign, n_arm); }
inline Vec3 touch_dir_to_arm(const WorkspaceMap& m, const Vec3& v_touch) { return flip(m.sign, v_touch); }

// ---- 2. stylus orientation -> wrist
// pitch = asin(axis_z): tilt of the stylus axis.         -> wrist_flex (via tool pitch)
// twist = rotation about the stylus axis since anchor.   -> wrist_roll
// twist: project a stylus-fixed vector (anchor b0, now b) onto the plane normal to axis a,
//        signed angle between the projections about a.
inline double stylus_pitch(const Vec3& axis) { return std::asin(clamp(axis[2], -1.0, 1.0)); }

inline double twist_about(const Vec3& a, const Vec3& b0, const Vec3& b) {
  const Vec3 p0 = sub(b0, mul(dot(b0, a), a));
  const Vec3 p1 = sub(b, mul(dot(b, a), a));
  return std::atan2(dot(a, cross(p0, p1)), dot(p0, p1));
}

// ---- 3. virtual wall (plane in the arm frame) with a proxy
// depth = (point - p) . n          > 0: inside (n points to free space)
// proxy = p + depth * n            closest surface point; the arm is sent HERE, never inside
// f     = max(0, k*depth - b*v_n)  felt at the stylus; never pulls
// Damper needed: a sampled spring adds energy each bounce (passive if b > k*T/2).
struct Wall {
  Vec3 point{0.0, 0.0, 0.05};
  Vec3 normal{0.0, 0.0, 1.0};  // unit (node normalizes)
  double stiffness = 150.0;    // N/m at the stylus
  double damping = 0.5;        // N/(m/s) at the stylus
};

struct WallProxy {
  double depth = 0.0;  // arm frame [m], > 0 = inside
  Vec3 proxy{};        // arm frame
};

inline WallProxy wall_proxy(const Wall& w, const Vec3& p) {
  WallProxy r;
  r.depth = dot(sub(w.point, p), w.normal);
  r.proxy = r.depth > 0.0 ? add(p, mul(r.depth, w.normal)) : p;
  return r;
}

// depth_touch [m], v_normal [m/s] in touch terms
inline double wall_force_mag(double k, double b, double depth_touch, double v_normal) {
  if (depth_touch <= 0.0) return 0.0;
  return std::max(0.0, k * depth_touch - b * v_normal);
}

// ---- 4. output safety filter (every force sent goes through this)
// NaN/inf -> return false, output 0 (caller latches a fault)
// |F| clamped to max_N, direction kept
// |F - F_prev| <= slew * dt, dt capped at 2 ms (a stalled tick can't allow a big jump)
inline bool safe_force(const Vec3& desired, const Vec3& prev, double max_N, double slew_N_per_s, double dt, Vec3& out) {
  if (!finite(desired) || !finite(prev)) {
    out = {0.0, 0.0, 0.0};
    return false;
  }
  const double mx = clamp(max_N, 0.0, kHardMaxForce_N);
  const double step = clamp(slew_N_per_s, 0.0, kHardMaxSlew_N_per_s) * clamp(dt, 0.0, 0.002);
  Vec3 target = desired;
  const double m = norm(target);
  if (m > mx) target = mul(mx / m, target);
  Vec3 d = sub(target, prev);
  const double dm = norm(d);
  if (dm > step) d = mul(step / dm, d);
  out = add(prev, d);
  const double om = norm(out);
  if (om > mx) out = mul(mx / om, out);
  return true;
}

// 1st-order low-pass: y += alpha * (x - y), alpha = dt / (dt + 1/(2*pi*fc))
inline double lowpass_alpha(double fc_hz, double dt) {
  if (fc_hz <= 0.0) return 1.0;
  return dt / (dt + 1.0 / (2.0 * kPi * fc_hz));
}

}  // namespace touch
