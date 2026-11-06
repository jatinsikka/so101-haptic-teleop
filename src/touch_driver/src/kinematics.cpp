// URDF -> PlanarArm. Walks base_link -> gripper_frame_link at q = 0, records each joint's
// position + axis, checks the planar structure, refuses otherwise.
// No KDL here: KDL uses Eigen, CHAI3D ships its own Eigen; keep them out of one program.
#include "touch_driver/kinematics.hpp"

#include <urdf/model.h>

#include <sstream>
#include <vector>

namespace touch {
namespace {

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};
Quat qmul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Vec3 qrot(const Quat& q, const Vec3& v) {  // q v q*
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = mul(2.0, cross(u, v));
  return add(add(v, mul(q.w, t)), cross(u, t));
}
struct Pose {
  Vec3 p{};
  Quat q{};
};
Pose compose(const Pose& a, const urdf::Pose& b) {
  const Quat qb{b.rotation.w, b.rotation.x, b.rotation.y, b.rotation.z};
  return {add(a.p, qrot(a.q, {b.position.x, b.position.y, b.position.z})), qmul(a.q, qb)};
}

}  // namespace

bool planar_from_urdf(const std::string& urdf_xml, PlanarArm& out, std::string& err) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    err = "could not parse robot_description";
    return false;
  }
  // tip -> root, then reverse
  std::vector<urdf::JointConstSharedPtr> chain;
  urdf::LinkConstSharedPtr link = model.getLink("gripper_frame_link");
  if (!link) {
    err = "URDF has no gripper_frame_link (is it uncommented in so101.urdf?)";
    return false;
  }
  while (link && link->name != "base_link") {
    if (!link->parent_joint) {
      err = "gripper_frame_link is not below base_link";
      return false;
    }
    chain.insert(chain.begin(), link->parent_joint);
    link = link->getParent();
  }

  // joint positions + axes in base_link at q = 0
  const char* names[5] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
  Vec3 O[5], A[5];
  int k = 0;
  Pose F;  // base_link
  for (const auto& j : chain) {
    F = compose(F, j->parent_to_joint_origin_transform);  // now at the joint frame
    if (j->type == urdf::Joint::FIXED) continue;
    if (k >= 5 || j->name != names[k]) {
      err = "unexpected joint '" + j->name + "' (expected " + (k < 5 ? names[k] : "none") + ")";
      return false;
    }
    O[k] = F.p;
    A[k] = qrot(F.q, {j->axis.x, j->axis.y, j->axis.z});  // URDF axis is in the joint frame
    if (!j->limits) {
      err = j->name + " has no <limit>";
      return false;
    }
    out.lo[k] = j->limits->lower;
    out.hi[k] = j->limits->upper;
    ++k;
  }
  if (k != 5) {
    err = "expected 5 moving joints, found " + std::to_string(k);
    return false;
  }
  const Vec3 tip = F.p;

  // ---- check structure, build the planar model
  std::ostringstream why;
  if (std::fabs(A[0][2]) < 0.999) why << "shoulder_pan axis is not vertical; ";
  for (int i = 1; i <= 3; ++i)
    if (std::fabs(A[i][1]) < 0.999) why << names[i] << " axis is not along y at the zero pose; ";
  if (!why.str().empty()) {
    err = why.str();
    return false;
  }
  out.c0 = A[0][2] > 0 ? 1.0 : -1.0;
  out.pan_x = O[0][0];
  out.pan_y = O[0][1];
  // +y rotation is clockwise in the (x, z) view -> alpha = -sign(axis_y) * q
  for (int i = 1; i <= 3; ++i) out.c[i] = A[i][1] > 0 ? -1.0 : 1.0;

  // tool point = gripper frame projected onto the roll axis (roll can't move it)
  const Vec3 T = add(O[4], mul(dot(sub(tip, O[4]), A[4]), A[4]));
  auto plane = [&](const Vec3& p) { return Vec2{p[0] - out.pan_x, p[2]}; };
  out.S = plane(O[1]);
  out.E = plane(O[2]);
  out.W = plane(O[3]);
  out.T = plane(T);
  // Plane offset from the tool point (S, E, W lie on sideways axes: their y is arbitrary).
  // Roll axis must lie in the plane.
  out.d = T[1] - out.pan_y;
  if (std::fabs(A[4][1]) > 0.01) {
    err = "wrist_roll axis is not in the arm's vertical plane";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- full model (drawing)
namespace {
Pose3 to_pose3(const urdf::Pose& u) {
  const double w = u.rotation.w, x = u.rotation.x, y = u.rotation.y, z = u.rotation.z;
  Pose3 o;
  o.p = {u.position.x, u.position.y, u.position.z};
  o.R = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
         2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
         2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
  return o;
}
Pose3 axis_rotation(const Vec3& a, double t) {  // Rodrigues
  const double c = std::cos(t), s = std::sin(t), k = 1 - c;
  Pose3 o;
  o.R = {c + a[0] * a[0] * k, a[0] * a[1] * k - a[2] * s, a[0] * a[2] * k + a[1] * s,
         a[1] * a[0] * k + a[2] * s, c + a[1] * a[1] * k, a[1] * a[2] * k - a[0] * s,
         a[2] * a[0] * k - a[1] * s, a[2] * a[1] * k + a[0] * s, c + a[2] * a[2] * k};
  return o;
}
}  // namespace

Pose3 compose(const Pose3& a, const Pose3& b) {
  Pose3 o;
  for (int r = 0; r < 3; ++r) {
    o.p[r] = a.p[r];
    for (int c = 0; c < 3; ++c) {
      o.p[r] += a.R[3 * r + c] * b.p[c];
      o.R[3 * r + c] = 0;
      for (int k = 0; k < 3; ++k) o.R[3 * r + c] += a.R[3 * r + k] * b.R[3 * k + c];
    }
  }
  return o;
}

bool robot_from_urdf(const std::string& urdf_xml, RobotModel& out, std::string& err) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    err = "could not parse robot_description";
    return false;
  }
  const char* names[6] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
  out = RobotModel{};
  // breadth-first from the root: parents before children
  std::vector<std::pair<urdf::LinkConstSharedPtr, int>> todo{{model.getRoot(), -1}};
  for (size_t i = 0; i < todo.size(); ++i) {
    const auto [link, parent] = todo[i];
    RobotModel::Link L;
    L.name = link->name;
    L.parent = parent;
    if (const auto& j = link->parent_joint) {
      L.origin = to_pose3(j->parent_to_joint_origin_transform);
      L.axis = {j->axis.x, j->axis.y, j->axis.z};
      for (int k = 0; k < 6; ++k)
        if (j->type != urdf::Joint::FIXED && j->name == names[k]) L.joint = k;
    }
    const int idx = static_cast<int>(out.links.size());
    out.links.push_back(L);
    for (const auto& v : link->visual_array) {
      if (!v || !v->geometry || v->geometry->type != urdf::Geometry::MESH) continue;
      MeshPart part;
      std::string f = std::static_pointer_cast<const urdf::Mesh>(v->geometry)->filename;
      const std::string pre = "package://so101_description/";
      if (f.rfind(pre, 0) == 0) f = f.substr(pre.size());
      part.file = f;
      part.link = idx;
      part.origin = to_pose3(v->origin);
      if (v->material)
        part.rgba = {static_cast<float>(v->material->color.r), static_cast<float>(v->material->color.g),
                     static_cast<float>(v->material->color.b), static_cast<float>(v->material->color.a)};
      out.parts.push_back(part);
    }
    for (const auto& child : link->child_links) todo.push_back({child, idx});
  }
  if (out.parts.empty()) {
    err = "URDF has no mesh visuals";
    return false;
  }
  return true;
}

std::vector<Pose3> link_poses(const RobotModel& m, const Arm5& q, double gripper) {
  std::vector<Pose3> T(m.links.size());
  for (size_t i = 0; i < m.links.size(); ++i) {
    const auto& L = m.links[i];
    if (L.parent < 0) continue;  // root = identity
    Pose3 t = compose(T[L.parent], L.origin);
    if (L.joint >= 0) t = compose(t, axis_rotation(L.axis, L.joint < 5 ? q[L.joint] : gripper));
    T[i] = t;
  }
  return T;
}

}  // namespace touch
