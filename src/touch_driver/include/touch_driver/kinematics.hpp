// SO-101 analytic FK/IK. Geometry read from the URDF (kinematics.cpp); checked vs KDL in
// test/test_kinematics.cpp.
//
// Model: planar 3-link arm (lift, elbow, wrist_flex: parallel axes) on a turntable (pan).
//   wrist_roll spins about the gripper axis; the tool point is ON that axis, so roll can't move it.
// IK (wrist-center: the hand sets the wrist point W, the tilt sets the tool pitch):
//   1. pan from atan2 of W seen from above
//   2. in the arm plane: shoulder->W by the law of cosines
//   3. wrist_flex = pitch - lift - elbow
//
// Plane coords (r, z): r = horizontal distance from the pan axis.
// alpha_i = c_i * q_i (c_i = +-1 from the URDF axis direction), CCW in (r, z).
// phi = alpha1 + alpha2 + alpha3 = tool pitch relative to the zero pose.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "touch_driver/haptic_math.hpp"

namespace touch {

using Vec2 = std::array<double, 2>;  // (r, z)

inline Vec2 rot2(double a, const Vec2& v) {
  const double c = std::cos(a), s = std::sin(a);
  return {c * v[0] - s * v[1], s * v[0] + c * v[1]};
}
inline Vec2 add2(const Vec2& a, const Vec2& b) { return {a[0] + b[0], a[1] + b[1]}; }
inline Vec2 sub2(const Vec2& a, const Vec2& b) { return {a[0] - b[0], a[1] - b[1]}; }
inline double norm2(const Vec2& a) { return std::hypot(a[0], a[1]); }

struct PlanarArm {
  double pan_x = 0, pan_y = 0;  // pan axis (vertical)
  double c0 = 1;                // +1 if the pan axis is +z
  double d = 0;                 // sideways offset of the arm plane from the pan axis
  Vec2 S{}, E{}, W{}, T{};      // zero pose: lift axis, elbow axis, wrist_flex axis, tool point
  std::array<double, 4> c{1, 1, 1, 1};  // c[1..3]: lift, elbow, wrist_flex
  Arm5 lo{}, hi{};                      // URDF joint limits [rad]
};

// Arm points in base_link.
struct ArmPose {
  Vec3 S{}, E{}, W{}, T{};  // T = tool point
  double phi = 0;           // tool pitch vs zero pose
};

inline Vec3 plane_to_3d(const PlanarArm& a, double alpha0, const Vec2& p) {
  const double c = std::cos(alpha0), s = std::sin(alpha0);
  return {a.pan_x + p[0] * c - a.d * s, a.pan_y + p[0] * s + a.d * c, p[1]};
}

inline ArmPose forward(const PlanarArm& a, const Arm5& q) {
  const double a0 = a.c0 * q[0];
  const double a1 = a.c[1] * q[1], a2 = a.c[2] * q[2], a3 = a.c[3] * q[3];
  const Vec2 E = add2(a.S, rot2(a1, sub2(a.E, a.S)));
  const Vec2 W = add2(E, rot2(a1 + a2, sub2(a.W, a.E)));
  const Vec2 T = add2(W, rot2(a1 + a2 + a3, sub2(a.T, a.W)));
  ArmPose p;
  p.S = plane_to_3d(a, a0, a.S);
  p.E = plane_to_3d(a, a0, E);
  p.W = plane_to_3d(a, a0, W);
  p.T = plane_to_3d(a, a0, T);
  p.phi = a1 + a2 + a3;
  return p;
}

struct IkResult {
  Arm5 q{};
  Vec3 reached{};        // FK(q).W; != target if clamped
  bool clamped = false;  // out of reach or joint limit hit
};

// Wrist-center IK. target: wrist point W (base_link). phi: tool pitch. roll: wrist_roll.
// Tilt (phi) only moves wrist_flex; shoulder/elbow depend on W alone.
// One solution only (no branch switching): pan faces the target, elbow bends the same way as at
// the zero pose. q_prev is only used when the target is above the pan axis.
inline IkResult inverse(const PlanarArm& a, const Vec3& target, double phi, double roll, const Arm5& q_prev) {
  IkResult res;
  Arm5& q = res.q;

  // ---- 1. pan: face the target
  const double dx = target[0] - a.pan_x, dy = target[1] - a.pan_y;
  const double rho2 = dx * dx + dy * dy;
  double alpha0 = a.c0 * q_prev[0];  // above the pan axis: keep the old pan
  if (rho2 - a.d * a.d > 0.02 * 0.02) alpha0 = std::atan2(dy, dx) - std::atan2(a.d, std::sqrt(rho2 - a.d * a.d));
  q[0] = clamp(alpha0 / a.c0, a.lo[0], a.hi[0]);
  alpha0 = a.c0 * q[0];
  const Vec2 P{dx * std::cos(alpha0) + dy * std::sin(alpha0), target[2]};  // target in the arm plane

  // ---- 2. shoulder -> wrist: 2-link, law of cosines
  const Vec2 L1 = sub2(a.E, a.S), L2 = sub2(a.W, a.E);
  const double l1 = norm2(L1), l2 = norm2(L2);
  const double b1 = std::atan2(L1[1], L1[0]), b2 = std::atan2(L2[1], L2[0]);
  Vec2 D = sub2(P, a.S);
  double n = norm2(D);
  // Keep the elbow >= 20 deg from straight: near straight (a singularity) a mm of reach needs a big
  // elbow move. Costs ~4 mm of reach.
  constexpr double kMinElbowBend = 0.35;
  const double n_max = std::sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * std::cos(kMinElbowBend));
  const double n_min = std::fabs(l1 - l2) + 1e-7;
  if (n > n_max || n < n_min) {  // out of reach -> nearest reachable distance, same direction
    const double nc = clamp(n, n_min, n_max);
    D = n > 1e-9 ? Vec2{D[0] * nc / n, D[1] * nc / n} : Vec2{nc, 0.0};
    n = nc;
  }
  const double elbow_sign = wrap_angle(b2 - b1) < 0 ? -1.0 : 1.0;  // bend direction at the zero pose
  const double delta = elbow_sign * std::acos(clamp((n * n - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0));
  const double g1 = std::atan2(D[1], D[0]) - std::atan2(l2 * std::sin(delta), l1 + l2 * std::cos(delta));
  const double a1 = wrap_angle(g1 - b1);
  const double a2 = wrap_angle(b1 + delta - b2);

  // ---- 3. wrist: whatever is left of the pitch
  const double a3 = wrap_angle(phi - a1 - a2);
  q[1] = clamp(a1 / a.c[1], a.lo[1], a.hi[1]);
  q[2] = clamp(a2 / a.c[2], a.lo[2], a.hi[2]);
  q[3] = clamp(a3 / a.c[3], a.lo[3], a.hi[3]);
  q[4] = clamp(roll, a.lo[4], a.hi[4]);

  res.reached = forward(a, q).W;
  res.clamped = norm(sub(res.reached, target)) > 1e-4;
  return res;
}

// From URDF text. false + reason if the structure above doesn't hold.
bool planar_from_urdf(const std::string& urdf_xml, PlanarArm& out, std::string& err);

// ---- full URDF model, for drawing the real meshes
struct Pose3 {
  Vec3 p{};
  std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major
};

struct MeshPart {
  std::string file;  // path relative to the so101_description share dir, e.g. "meshes/x.stl"
  int link = 0;
  Pose3 origin;      // visual origin in the link frame
  std::array<float, 4> rgba{0.7f, 0.7f, 0.7f, 1.0f};
};

struct RobotModel {
  struct Link {
    std::string name;
    int parent = -1;  // parents come before children
    Pose3 origin;     // parent joint origin
    Vec3 axis{};
    int joint = -1;   // 0..4 arm joints, 5 gripper, -1 fixed
  };
  std::vector<Link> links;
  std::vector<MeshPart> parts;
};

bool robot_from_urdf(const std::string& urdf_xml, RobotModel& out, std::string& err);
// World (base_link) pose of every link.
std::vector<Pose3> link_poses(const RobotModel& m, const Arm5& q, double gripper);
Pose3 compose(const Pose3& a, const Pose3& b);

}  // namespace touch
