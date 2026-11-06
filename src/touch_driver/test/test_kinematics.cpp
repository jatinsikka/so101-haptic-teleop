// Tests for kinematics.hpp: our hand-derived planar FK/IK vs KDL (an independent library that
// reads the same URDF). No device, no ROS. Exit code 0 = all passed.
//   1. FK matches KDL on random poses        -> the planar model of the URDF is right
//   2. IK(FK(q)) reaches the same point      -> the law-of-cosines solution is right
//   3. IK picks the branch near q_prev       -> no elbow flips while you move
//   4. unreachable targets give a finite, clamped answer
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "touch_driver/kinematics.hpp"

using namespace touch;

static int failures = 0;
#define CHECK(cond, ...)                                          \
  do {                                                            \
    if (!(cond)) {                                                \
      ++failures;                                                 \
      std::printf("FAIL %s:%d  %s  ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                   \
      std::printf("\n");                                          \
    }                                                             \
  } while (0)

// KDL's version of our "tool point": gripper frame projected onto the wrist_roll axis.
static Vec3 kdl_tool_point(const KDL::Chain& chain, const Arm5& q) {
  KDL::Frame F = KDL::Frame::Identity();
  KDL::Vector O4, A4;
  int k = 0;
  for (unsigned s = 0; s < chain.getNrOfSegments(); ++s) {
    const KDL::Segment& seg = chain.getSegment(s);
    const KDL::Joint& j = seg.getJoint();
    double qj = 0;
    if (j.getType() != KDL::Joint::None) {
      if (j.getName() == "wrist_roll") {
        O4 = F * j.JointOrigin();
        A4 = F.M * j.JointAxis();
      }
      qj = q[k++];
    }
    F = F * seg.pose(qj);
  }
  const KDL::Vector tip = F.p;
  const KDL::Vector T = O4 + KDL::dot(tip - O4, A4) * A4;
  return {T.x(), T.y(), T.z()};
}

int main() {
  std::ifstream f(SO101_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string xml = ss.str();

  PlanarArm arm;
  std::string err;
  if (!planar_from_urdf(xml, arm, err)) {
    std::printf("FAIL planar_from_urdf: %s\n", err.c_str());
    return 1;
  }
  std::printf("planar model: pan axis (%.4f, %.4f) c0=%+.0f d=%.4f\n", arm.pan_x, arm.pan_y, arm.c0, arm.d);
  std::printf("  S=(%.4f, %.4f) E=(%.4f, %.4f) W=(%.4f, %.4f) T=(%.4f, %.4f)  c=[%+.0f %+.0f %+.0f]\n", arm.S[0], arm.S[1],
              arm.E[0], arm.E[1], arm.W[0], arm.W[1], arm.T[0], arm.T[1], arm.c[1], arm.c[2], arm.c[3]);
  std::printf("  link lengths: upper %.4f  fore %.4f  wrist->tool %.4f m\n", norm2(sub2(arm.E, arm.S)),
              norm2(sub2(arm.W, arm.E)), norm2(sub2(arm.T, arm.W)));

  KDL::Tree tree;
  KDL::Chain chain;
  CHECK(kdl_parser::treeFromString(xml, tree), "kdl tree");
  CHECK(tree.getChain("base_link", "gripper_frame_link", chain), "kdl chain");

  std::mt19937 rng(7);
  auto rand_q = [&](double shrink) {
    Arm5 q;
    for (int i = 0; i < 5; ++i) {
      const double mid = 0.5 * (arm.lo[i] + arm.hi[i]), half = 0.5 * (arm.hi[i] - arm.lo[i]) * shrink;
      q[i] = std::uniform_real_distribution<double>(mid - half, mid + half)(rng);
    }
    return q;
  };

  // 1. FK vs KDL
  double worst = 0;
  for (int t = 0; t < 2000; ++t) {
    const Arm5 q = rand_q(1.0);
    worst = std::max(worst, norm(sub(forward(arm, q).T, kdl_tool_point(chain, q))));
  }
  // URDF uses 1.5708, not pi/2: axes are ~1e-5 rad off parallel -> a few um. Servo resolution ~0.1 mm.
  CHECK(worst < 1e-4, "FK differs from KDL by %.3e m", worst);
  std::printf("FK vs KDL: worst error %.2e m over 2000 poses\n", worst);

  // 1b. full model (used to draw the meshes): gripper_frame_link pose vs KDL
  RobotModel model;
  CHECK(robot_from_urdf(xml, model, err), "robot_from_urdf: %s", err.c_str());
  int tip_idx = -1;
  for (size_t i = 0; i < model.links.size(); ++i)
    if (model.links[i].name == "gripper_frame_link") tip_idx = static_cast<int>(i);
  CHECK(tip_idx >= 0, "no gripper_frame_link in model");
  KDL::ChainFkSolverPos_recursive fk(chain);
  double worst_p = 0, worst_R = 0;
  for (int t = 0; t < 500 && tip_idx >= 0; ++t) {
    const Arm5 q = rand_q(1.0);
    KDL::JntArray jq(5);
    for (int i = 0; i < 5; ++i) jq(i) = q[i];
    KDL::Frame F;
    fk.JntToCart(jq, F);
    const Pose3 P = link_poses(model, q, 0.3)[tip_idx];
    worst_p = std::max(worst_p, norm(sub(P.p, {F.p.x(), F.p.y(), F.p.z()})));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) worst_R = std::max(worst_R, std::fabs(P.R[3 * r + c] - F.M(r, c)));
  }
  CHECK(worst_p < 1e-9 && worst_R < 1e-9, "mesh model vs KDL: %.2e m, %.2e", worst_p, worst_R);
  std::printf("mesh model: %zu links, %zu mesh parts, tip vs KDL %.1e m\n", model.links.size(), model.parts.size(), worst_p);

  // 2. IK round trip on the WRIST point, for poses IK is meant to produce:
  //    pan facing the wrist (r > 0) and the elbow bent the same way as at the zero pose.
  const Vec2 L1 = sub2(arm.E, arm.S), L2 = sub2(arm.W, arm.E);
  const double b1 = std::atan2(L1[1], L1[0]), b2 = std::atan2(L2[1], L2[0]);
  const double elbow_sign = wrap_angle(b2 - b1) < 0 ? -1.0 : 1.0;
  int tested = 0, bad = 0;
  for (int t = 0; t < 4000; ++t) {
    const Arm5 q = rand_q(0.9);
    const ArmPose p = forward(arm, q);
    const double a0 = arm.c0 * q[0];
    const double r = (p.W[0] - arm.pan_x) * std::cos(a0) + (p.W[1] - arm.pan_y) * std::sin(a0);
    const double bend = wrap_angle(b2 + arm.c[2] * q[2] - b1);
    if (r < 0.03 || bend * elbow_sign < 0.36) continue;  // not a pose this IK produces (>= 20 deg bend)
    ++tested;
    const IkResult res = inverse(arm, p.W, p.phi, q[4], Arm5{});
    double dq = 0;
    for (int i = 0; i < 5; ++i) dq = std::max(dq, std::fabs(res.q[i] - q[i]));
    if (norm(sub(res.reached, p.W)) > 1e-6 || dq > 1e-6) ++bad;
  }
  CHECK(bad == 0, "%d of %d poses: IK(FK(q).W) != q", bad, tested);
  std::printf("IK round trip: %d poses\n", tested);

  // 3. no flips: sweep the wrist target along a line through and beyond the workspace, 1 mm steps.
  //    Every joint must change smoothly (< 0.05 rad per mm) - this is what "erratic" looked like.
  {
    Arm5 q_prev{0, -0.8, 0.8, 0, 0};
    const Vec3 A{0.10, -0.30, 0.00}, B{0.60, 0.30, 0.45};
    const int steps = static_cast<int>(norm(sub(B, A)) / 0.001);
    double worst = 0;
    for (int k = 0; k <= steps; ++k) {
      const Vec3 w = add(A, mul(static_cast<double>(k) / steps, sub(B, A)));
      const IkResult res = inverse(arm, w, 0.0, 0.0, q_prev);
      if (k > 0)
        for (int i = 0; i < 4; ++i) {
          const double d = std::fabs(res.q[i] - q_prev[i]);
          if (d > 0.05)
            std::printf("  jump: step %d joint %d  %.3f -> %.3f  at (%.3f %.3f %.3f) clamped=%d\n", k, i, q_prev[i],
                        res.q[i], w[0], w[1], w[2], res.clamped);
          worst = std::max(worst, d);
        }
      q_prev = res.q;
    }
    CHECK(worst < 0.05, "joint jumped %.3f rad in one 1 mm step", worst);
    std::printf("sweep: %d steps, largest joint step %.4f rad/mm\n", steps, worst);
  }

  // 4. unreachable: 2 m away -> finite, clamped, and pointing the right way
  const IkResult far = inverse(arm, {2.0, 0.5, 0.3}, 0.0, 0.0, Arm5{});
  CHECK(far.clamped && finite(far.reached), "far target");
  CHECK(far.reached[0] > 0.1 && far.reached[1] > 0.0, "far target reached (%.3f %.3f %.3f)", far.reached[0],
        far.reached[1], far.reached[2]);

  // a few poses, to choose a sensible home (printed, not tested)
  for (const Arm5& q : {Arm5{0, 0, 0, 0, 0}, Arm5{0, -0.8, 0.8, 0, 0}, Arm5{0, 0.8, -0.8, 0, 0}, Arm5{0, -1.0, 1.4, 0.6, 0}}) {
    const ArmPose p = forward(arm, q);
    std::printf("  q=[%+.1f %+.1f %+.1f %+.1f %+.1f] -> tool (%.3f, %.3f, %.3f)  E z=%.3f\n", q[0], q[1], q[2], q[3], q[4],
                p.T[0], p.T[1], p.T[2], p.E[2]);
  }

  std::printf(failures ? "%d FAILURE(S)\n" : "all kinematics tests passed\n", failures);
  return failures ? 1 : 0;
}
