// Touch via CHAI3D: haptic loop + window. Structure from CHAI3D examples/GLFW/01-mydevice.
//
// CHAI3D's Phantom driver does NOT clamp forces, check errors, or report calibration
// (and getPosition() returns false on success). All safety is here + haptic_math.hpp:
//   1. enable_force=false -> zero force only
//   2. deadman: button 1 held, fresh press only
//   3. engage ramp
//   4. follower watchdog
//   5. safe_force: NaN -> fault, clamp, slew
//   6. fault latch until /touch/clear_fault
//   7. shutdown: zero force, stop loop, then close device
// Pacing: setForce...() waits for the device's 1 kHz servo tick; no sleep needed.
#include "touch_driver/haptics.hpp"

#include <cstdio>
#include <sstream>

#include "chai3d.h"
#include <GLFW/glfw3.h>

namespace touch {

using namespace chai3d;

namespace {
cVector3d cv(const Vec3& v) { return cVector3d(v[0], v[1], v[2]); }
Vec3 vv(const cVector3d& v) { return {v.x(), v.y(), v.z()}; }
const cVector3d kZero(0, 0, 0);

// One tick's reading of the Touch (touch frame).
struct Reading {
  Vec3 p{}, v{};        // position, velocity
  Vec3 axis{}, side{};  // stylus axis, a vector perpendicular to it (for twist)
  bool button1 = false;
  bool ok = true;       // all finite
};

// The SO-101 drawn from its URDF meshes. One cMultiMesh per <visual>, posed every frame.
struct MeshArm {
  const RobotModel* model = nullptr;
  std::vector<cMultiMesh*> parts;  // nullptr where a mesh failed to load

  // color: nullptr = URDF colors. alpha < 1 = see-through. Returns meshes loaded.
  int create(cWorld* world, const RobotModel& m, const std::string& dir, const float* color, float alpha) {
    model = &m;
    int loaded = 0;
    for (const MeshPart& p : m.parts) {
      auto* mm = new cMultiMesh();
      if (!mm->loadFromFile(dir + "/" + p.file)) {
        delete mm;
        parts.push_back(nullptr);
        continue;
      }
      cMaterial mat;
      if (color) mat.setColorf(color[0], color[1], color[2]);
      else mat.setColorf(p.rgba[0], p.rgba[1], p.rgba[2]);
      mm->setUseVertexColors(false, true);
      mm->setMaterial(mat, true);
      if (alpha < 1.0f) {
        mm->setUseTransparency(true, true);
        mm->setTransparencyLevel(alpha, false, false, true);
      }
      world->addChild(mm);
      parts.push_back(mm);
      ++loaded;
    }
    return loaded;
  }
  void update(const Arm5& q, bool show) {
    const std::vector<Pose3> T = link_poses(*model, q, 0.0);  // gripper closed (Hapkit drives it)
    for (size_t i = 0; i < parts.size(); ++i) {
      if (!parts[i]) continue;
      const Pose3 W = compose(T[model->parts[i].link], model->parts[i].origin);
      parts[i]->setLocalPos(cv(W.p));
      parts[i]->setLocalRot(cMatrix3d(W.R[0], W.R[1], W.R[2], W.R[3], W.R[4], W.R[5], W.R[6], W.R[7], W.R[8]));
      parts[i]->setShowEnabled(show);
    }
  }
};

// Stick-figure arm (fallback if the meshes can't be loaded): base, shoulder, elbow, wrist, tool.
struct StickArm {
  cShapeLine* seg[4]{};
  cShapeSphere* joint[4]{};
  void create(cWorld* world, float r, float g, float b, double width) {
    for (auto& s : seg) {
      s = new cShapeLine(kZero, kZero);
      s->setLineWidth(width);
      s->m_colorPointA.set(r, g, b);
      s->m_colorPointB.set(r, g, b);
      world->addChild(s);
    }
    for (auto& j : joint) {
      j = new cShapeSphere(0.006);
      j->m_material->setColorf(r, g, b);
      world->addChild(j);
    }
  }
  void update(const PlanarArm& arm, const Arm5& q, bool show) {
    const ArmPose p = forward(arm, q);
    const Vec3 pts[5] = {{arm.pan_x, arm.pan_y, 0.0}, p.S, p.E, p.W, p.T};
    for (int i = 0; i < 4; ++i) {
      seg[i]->m_pointA = cv(pts[i]);
      seg[i]->m_pointB = cv(pts[i + 1]);
      joint[i]->setLocalPos(cv(pts[i + 1]));
      seg[i]->setShowEnabled(show);
      joint[i]->setShowEnabled(show);
    }
  }
};
}  // namespace

struct HapticApp::Impl {
  Shared& shared;
  const Config cfg;
  const PlanarArm arm;
  const RobotModel model;

  cHapticDeviceHandler* handler = nullptr;
  cGenericHapticDevicePtr device;
  bool device_open = false;
  cThread* thread = nullptr;
  std::atomic<bool> running{false}, finished{true};

  // ---- haptic loop state (haptic thread only)
  struct Loop {
    FromRos in;                // last copy from ROS
    bool follower_live = false;
    bool engaged = false;
    bool need_release = true;  // button already down at startup doesn't count
    bool fault = false;
    Fault reason = Fault::kNone;
    double ramp = 0;
    WorkspaceMap map;          // anchors set on engage
    double pitch0 = 0, phi0 = 0, roll0 = 0;
    Vec3 side0{1, 0, 0};
    double twist = 0, twist_prev = 0;
    Arm5 q_target{};           // rate-limited (what the arm is told)
    Vec3 p_hand{}, p_proxy{}, p_reached{};  // wrist points: hand, after wall, reached by q_target
    Vec3 p_tool{};             // tool tip of q_target
    Vec3 p_ik{};               // wrist point the IK can reach (before the rate limit)
    Vec3 v_filt{};             // low-passed stylus velocity (for damping)
    double depth = 0;          // wall penetration of the tool tip (arm frame)
    bool clamped = false;
    Vec3 fb_filt{}, F_prev{};
    double rate_hz = 0;
    std::uint64_t ticks = 0;
  } L;

  // ---- graphics
  GLFWwindow* window = nullptr;
  int fb_w = 0, fb_h = 0;
  cWorld* world = nullptr;
  cCamera* camera = nullptr;
  cViewport* viewport = nullptr;
  MeshArm ghost_mesh, follower_mesh;
  StickArm ghost, follower;
  bool use_meshes = false;
  cShapeSphere *hand = nullptr, *tool = nullptr;
  cShapeLine *spring = nullptr, *force_arrow = nullptr;
  cMesh* wall_mesh = nullptr;
  cLabel *status = nullptr, *info = nullptr, *help = nullptr, *banner = nullptr;
  cFontPtr font;
  cFrequencyCounter graphics_rate;
  bool engaged_now = false;  // keys only act when not engaged

  Impl(Shared& s, const Config& c, const PlanarArm& a, const RobotModel& m) : shared(s), cfg(c), arm(a), model(m) {}

  // ================================================================ haptic loop
  static void trampoline(void* self) { static_cast<Impl*>(self)->haptic_loop(); }

  void haptic_loop() {
    finished = false;
    L.map = cfg.map;
    L.q_target = cfg.home;
    L.p_hand = L.p_proxy = L.p_reached = forward(arm, L.q_target).W;
    Clock::time_point t_prev = Clock::now(), t_rate = t_prev;
    std::uint64_t ticks_at_rate = 0;

    while (running) {
      const Clock::time_point now = Clock::now();
      const double dt = std::chrono::duration<double>(now - t_prev).count();
      t_prev = now;

      const Reading r = read_touch();       // 1
      read_ros(now, r.button1);             // 2
      deadman(r);                           // 3
      update_target(r, dt);                 // 4
      const Vec3 F = compute_force(r, dt);  // 5
      const Vec3 F_out = output(F, dt);     // 6
      publish_state(F_out, r);              // 7

      ++L.ticks;
      const double since = std::chrono::duration<double>(now - t_rate).count();
      if (since >= 0.5) {
        L.rate_hz = (L.ticks - ticks_at_rate) / since;
        ticks_at_rate = L.ticks;
        t_rate = now;
      }
    }
    // shutdown: zero force for 10 ticks before the device is closed
    for (int i = 0; i < 10; ++i) device->setForceAndTorqueAndGripperForce(kZero, kZero, 0.0);
    finished = true;
  }

  // 1. read the Touch. Bad values latch a fault.
  Reading read_touch() {
    cVector3d pos, vel;
    cMatrix3d rot;
    unsigned buttons = 0;
    device->getPosition(pos);
    device->getRotation(rot);
    device->getLinearVelocity(vel);
    device->getUserSwitches(buttons);
    const cVector3d cols[3] = {rot.getCol0(), rot.getCol1(), rot.getCol2()};
    Reading r;
    r.p = vv(pos);
    r.v = vv(vel);
    r.axis = vv(cols[cfg.stylus_axis % 3]);
    r.side = vv(cols[(cfg.stylus_axis + 1) % 3]);
    r.button1 = (buttons & 0x1) != 0;
    r.ok = finite(r.p) && finite(r.v) && finite(r.axis) && finite(r.side);
    if (!r.ok) latch(Fault::kBadReading);
    return r;
  }

  // 2. ROS input (try_lock: never wait), watchdog, fault clear.
  void read_ros(Clock::time_point now, bool button1) {
    if (shared.from_ros_mtx.try_lock()) {
      L.in = shared.from_ros;
      shared.from_ros.clear_fault = false;
      shared.from_ros_mtx.unlock();
    }
    L.follower_live = L.in.have_follower && std::chrono::duration<double>(now - L.in.stamp).count() < cfg.watchdog_s;
    if (L.in.clear_fault) {
      if (L.fault && !button1) {  // a still-bad reading re-latches next tick
        L.fault = false;
        L.reason = Fault::kNone;
        L.need_release = true;
      }
      L.in.clear_fault = false;
    }
  }

  // 3. deadman + clutch. On a fresh press: anchor stylus pose to the real arm pose.
  void deadman(const Reading& r) {
    if (L.fault || !r.button1) {
      if (!r.button1) L.need_release = false;
      L.engaged = false;
    } else if (!L.engaged && !L.need_release) {
      L.engaged = true;
      L.ramp = 0.0;
      if (L.follower_live)  // anchor at the real arm -> no jump
        for (int i = 0; i < 5; ++i) L.q_target[i] = clamp(L.in.q_follower[i], arm.lo[i], arm.hi[i]);
      const ArmPose ap = forward(arm, L.q_target);
      L.map.touch_anchor = r.p;
      L.map.arm_anchor = ap.W;  // the hand drives the wrist point
      L.pitch0 = stylus_pitch(r.axis);
      L.phi0 = ap.phi;
      L.roll0 = L.q_target[4];
      L.side0 = r.side;
      L.twist = L.twist_prev = 0.0;
      L.fb_filt = {0, 0, 0};
    }
  }

  // 4. arm target: map -> IK -> wall (on the tool tip) -> rate limit.
  //    Not engaged: ghost sits on the real arm.
  void update_target(const Reading& r, double dt) {
    L.depth = 0.0;
    L.clamped = false;
    if (!L.engaged) {
      if (L.follower_live) L.q_target = L.in.q_follower;
      const ArmPose ap = forward(arm, L.q_target);
      L.p_hand = L.p_proxy = L.p_reached = L.p_ik = ap.W;
      L.p_tool = ap.T;
      return;
    }
    const double tw = twist_about(r.axis, L.side0, r.side);
    L.twist += wrap_angle(tw - L.twist_prev);  // unwrap
    L.twist_prev = tw;
    const double phi = L.phi0 + cfg.pitch_gain * (stylus_pitch(r.axis) - L.pitch0);
    const double roll = L.roll0 + cfg.roll_gain * L.twist;

    L.p_hand = touch_to_arm(L.map, r.p);  // desired wrist point
    L.p_proxy = L.p_hand;
    IkResult ik = inverse(arm, L.p_hand, phi, roll, L.q_target);
    if (shared.wall_on) {  // wall acts on the tool TIP: if inside, lift the whole tool out
      const WallProxy wp = wall_proxy(cfg.wall, forward(arm, ik.q).T);
      if (wp.depth > 0.0) {
        L.depth = wp.depth;
        L.p_proxy = add(L.p_hand, mul(wp.depth, cfg.wall.normal));
        ik = inverse(arm, L.p_proxy, phi, roll, L.q_target);
      }
    }
    L.clamped = ik.clamped;
    L.p_ik = ik.reached;

    // joint rate limit: no IK jump reaches the arm faster than max_joint_speed
    const double step = cfg.max_joint_speed * clamp(dt, 0.0, 0.002);
    for (int i = 0; i < 5; ++i) L.q_target[i] += clamp(ik.q[i] - L.q_target[i], -step, step);
    const ArmPose ap = forward(arm, L.q_target);
    L.p_reached = ap.W;
    L.p_tool = ap.T;
  }

  // 5. desired force (touch frame, stiffness as felt at the stylus), before the safety filter.
  Vec3 compute_force(const Reading& r, double dt) {
    const double dtc = clamp(dt, 1e-4, 0.002);
    // velocity estimate is noisy; noise x damping = "sandpaper". Low-pass it first.
    L.v_filt = add(L.v_filt, mul(lowpass_alpha(cfg.velocity_lpf_hz, dtc), sub(r.v, L.v_filt)));
    if (!L.engaged) return {0, 0, 0};
    const Vec3& v = L.v_filt;
    Vec3 F = mul(-cfg.free_damping, v);    // damping
    if (L.depth > 0.0) {                   // wall
      const Vec3 n = arm_dir_to_touch(L.map, cfg.wall.normal);
      const double f = wall_force_mag(cfg.wall.stiffness, cfg.wall.damping, L.depth / L.map.scale, dot(v, n));
      F = add(F, mul(f, n));
    }
    // reach / joint limits: only where the IK itself can't go (not the rate-limit lag, which steps every tick)
    if (cfg.reach_enabled)
      F = add(F, mul(cfg.reach_stiffness, arm_delta_to_touch(L.map, sub(L.p_ik, L.p_proxy))));

    Vec3 fb{0, 0, 0};                      // follower feedback (deadband, low-pass)
    if (shared.feedback_on && L.follower_live) {
      const Vec3 e = sub(forward(arm, L.in.q_follower).T, L.p_reached);
      const double m = norm(e);
      if (m > cfg.feedback_deadband_m)
        fb = mul(cfg.feedback_stiffness, arm_delta_to_touch(L.map, mul((m - cfg.feedback_deadband_m) / m, e)));
    }
    L.fb_filt = add(L.fb_filt, mul(lowpass_alpha(cfg.feedback_lpf_hz, dtc), sub(fb, L.fb_filt)));
    F = add(F, L.fb_filt);

    L.ramp = std::min(1.0, L.ramp + dt / cfg.engage_ramp_s);  // engage ramp
    return mul(L.ramp, F);
  }

  // 6. safety filter -> device. Dropping to zero is not slew-limited (can't kick).
  Vec3 output(const Vec3& F, double dt) {
    Vec3 out{0, 0, 0};
    if (cfg.enable_force && L.engaged && !L.fault && !safe_force(F, L.F_prev, cfg.max_force_N, cfg.slew_N_per_s, dt, out))
      latch(Fault::kBadForce);  // out is zero
    L.F_prev = out;
    device->setForceAndTorqueAndGripperForce(cv(out), kZero, 0.0);  // blocks until servo tick
    return out;
  }

  // 7. snapshot for ROS + graphics (try_lock: never wait).
  void publish_state(const Vec3& F_out, const Reading& r) {
    if (!shared.state_mtx.try_lock()) return;
    HapticState& s = shared.state;
    s.ticks = L.ticks;
    s.rate_hz = L.rate_hz;
    s.engaged = L.engaged;
    s.fault = L.fault;
    s.fault_reason = L.reason;
    s.follower_live = L.follower_live;
    s.in_wall = L.depth > 0.0;
    s.clamped = L.clamped;
    s.wall_on = shared.wall_on;
    s.feedback_on = shared.feedback_on;
    s.force = F_out;
    s.pitch = stylus_pitch(r.axis);
    s.twist = L.twist;
    s.q_target = L.q_target;
    s.q_follower = L.in.q_follower;
    s.p_hand = L.p_hand;
    s.p_reached = L.p_reached;
    s.p_tool = L.p_tool;
    shared.state_mtx.unlock();
  }

  void latch(Fault f) {
    L.fault = true;
    L.reason = f;
    L.engaged = false;
  }

  // ================================================================ graphics
  void build_scene() {
    world = new cWorld();
    world->m_backgroundColor.set(0.12f, 0.13f, 0.15f);
    camera = new cCamera(world);
    world->addChild(camera);
    // from the operator's seat: behind the base, looking the way the arm faces
    camera->set(cVector3d(-0.45, 0.0, 0.40), cVector3d(0.20, 0.0, 0.10), cVector3d(0, 0, 1));
    camera->setClippingPlanes(0.01, 10.0);
    auto* light = new cDirectionalLight(world);
    world->addChild(light);
    light->setEnabled(true);
    light->setDir(-1.0, 0.5, -1.0);

    auto* table = new cMesh();  // z = 0
    cCreatePlane(table, 0.9, 0.9, cVector3d(0.2, 0, -0.001));
    table->m_material->setColorf(0.30f, 0.30f, 0.32f);
    table->setUseCulling(false);
    world->addChild(table);

    // wall: translucent square, +z along the normal
    const Vec3 n = cfg.wall.normal;
    Vec3 u = cross(std::fabs(n[2]) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, n);
    u = mul(1.0 / norm(u), u);
    cMatrix3d R;
    R.setCol(cv(u), cv(cross(n, u)), cv(n));
    wall_mesh = new cMesh();
    cCreatePlane(wall_mesh, 0.5, 0.5, cv(cfg.wall.point), R);
    wall_mesh->m_material->setColorf(0.9f, 0.3f, 0.3f);
    wall_mesh->setUseCulling(false);
    wall_mesh->setUseTransparency(true);
    wall_mesh->setTransparencyLevel(0.35f);
    world->addChild(wall_mesh);

    // real arm: solid, URDF colors. ghost (target): see-through blue. Fallback: stick figures.
    static const float kBlue[3] = {0.35f, 0.75f, 1.0f};
    const int n_follower = follower_mesh.create(world, model, cfg.description_dir, nullptr, 1.0f);
    const int n_ghost = ghost_mesh.create(world, model, cfg.description_dir, kBlue, 0.35f);
    use_meshes = n_follower > 0 && n_ghost > 0;
    if (use_meshes) {
      std::printf("[touch] loaded %d SO-101 meshes from %s\n", n_ghost, cfg.description_dir.c_str());
    } else {
      std::printf("[touch] no meshes found in '%s' -> drawing stick figures\n", cfg.description_dir.c_str());
      follower.create(world, 0.95f, 0.60f, 0.20f, 7.0);  // orange: real arm
      ghost.create(world, 0.35f, 0.75f, 1.00f, 3.0);     // blue: target
    }
    hand = new cShapeSphere(0.007);                    // yellow: hand
    hand->m_material->setColorf(1.0f, 0.85f, 0.2f);
    world->addChild(hand);
    tool = new cShapeSphere(0.010);                    // tool tip (red in wall)
    world->addChild(tool);
    spring = new cShapeLine(kZero, kZero);             // hand -> reached wrist (stretched = wall/reach)
    spring->m_colorPointA.set(1.0f, 0.85f, 0.2f);
    spring->m_colorPointB.set(0.35f, 0.75f, 1.0f);
    spring->setLineWidth(2.0);
    world->addChild(spring);
    force_arrow = new cShapeLine(kZero, kZero);
    force_arrow->m_colorPointA.set(1.0f, 0.2f, 0.2f);
    force_arrow->m_colorPointB.set(1.0f, 0.2f, 0.2f);
    force_arrow->setLineWidth(4.0);
    world->addChild(force_arrow);

    font = NEW_CFONT_CONSOLAS_18();
    status = new cLabel(font);
    info = new cLabel(font);
    help = new cLabel(font);
    banner = new cLabel(font);
    for (cLabel* l : {status, info, help, banner}) {
      l->m_fontColor.set(0.92f, 0.92f, 0.92f);
      camera->m_frontLayer->addChild(l);
    }
    banner->m_fontColor.set(1.0f, 0.35f, 0.35f);
    banner->setText("FORCES OFF (read-only): the Touch will not push. Relaunch with enable_force:=true");
    banner->setShowEnabled(!cfg.enable_force);
    help->setText("hold button 1 = engage (release = clutch)  |  [w] wall  [b] feedback (when not engaged)  |  [q] quit");
  }

  void update_scene() {
    HapticState s;
    {
      std::lock_guard<std::mutex> lk(shared.state_mtx);  // graphics may wait
      s = shared.state;
    }
    engaged_now = s.engaged;
    if (use_meshes) {
      ghost_mesh.update(s.q_target, true);
      follower_mesh.update(s.q_follower, s.follower_live);
    } else {
      ghost.update(arm, s.q_target, true);
      follower.update(arm, s.q_follower, s.follower_live);
    }
    hand->setLocalPos(cv(s.p_hand));
    hand->setShowEnabled(s.engaged);
    tool->setLocalPos(cv(s.p_tool));
    if (s.in_wall)
      tool->m_material->setColorf(1.0f, 0.3f, 0.3f);
    else
      tool->m_material->setColorf(0.35f, 0.75f, 1.0f);
    spring->m_pointA = cv(s.p_hand);
    spring->m_pointB = cv(s.p_reached);
    spring->setShowEnabled(s.engaged);
    force_arrow->m_pointA = cv(s.p_tool);  // arm frame, 10 cm/N
    force_arrow->m_pointB = cv(add(s.p_tool, mul(0.10, touch_dir_to_arm(cfg.map, s.force))));
    force_arrow->setShowEnabled(norm(s.force) > 1e-3);
    wall_mesh->setShowEnabled(s.wall_on);

    std::ostringstream st;
    st.setf(std::ios::fixed);
    st.precision(0);
    if (s.fault)
      st << "FAULT: " << fault_name(s.fault_reason) << "  (release button 1, ros2 service call /touch/clear_fault std_srvs/srv/Trigger)";
    else if (s.engaged)
      st << "ENGAGED" << (cfg.enable_force ? "" : " (forces disabled)") << (s.in_wall ? "  IN WALL" : "")
         << (s.clamped ? "  AT REACH/JOINT LIMIT" : "");
    else
      st << (cfg.enable_force ? "free: press button 1 to engage" : "read-only (forces disabled): press button 1 to move the ghost");
    st << "   |  haptics " << s.rate_hz << " Hz  graphics " << graphics_rate.getFrequency() << " Hz";
    status->setText(st.str());

    std::ostringstream nf;
    nf.setf(std::ios::fixed);
    nf.precision(3);
    nf << "force " << norm(s.force) << " N   pitch " << s.pitch << "  twist " << s.twist << " rad   follower "
       << (s.follower_live ? "live" : "not heard") << "   wall " << (s.wall_on ? "on" : "off") << "   feedback "
       << (s.feedback_on ? "on" : "off");
    info->setText(nf.str());

    const int h = viewport ? viewport->getDisplayHeight() : 600;
    status->setLocalPos(15, h - 30);
    info->setLocalPos(15, h - 55);
    help->setLocalPos(15, 12);
    banner->setLocalPos(15, h - 85);
  }

  static void on_key(GLFWwindow* w, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    auto* self = static_cast<Impl*>(glfwGetWindowUserPointer(w));
    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
      glfwSetWindowShouldClose(w, GLFW_TRUE);
      return;
    }
    if (key != GLFW_KEY_W && key != GLFW_KEY_B) return;
    if (self->engaged_now) {  // toggling while engaged could step the force
      std::printf("[touch] release button 1 before toggling\n");
      return;
    }
    std::atomic<bool>& flag = key == GLFW_KEY_W ? self->shared.wall_on : self->shared.feedback_on;
    flag = !flag;
    std::printf("[touch] %s %s\n", key == GLFW_KEY_W ? "wall" : "follower feedback", flag ? "ON" : "OFF");
  }
  static void on_fb_size(GLFWwindow* w, int width, int height) {
    auto* self = static_cast<Impl*>(glfwGetWindowUserPointer(w));
    self->fb_w = width;
    self->fb_h = height;
  }

  bool open_window() {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    window = glfwCreateWindow(1100, 700, "SO-101 teleop - Geomagic Touch", nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      return false;
    }
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, on_key);
    glfwSetFramebufferSizeCallback(window, on_fb_size);
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) return false;
#endif
    build_scene();
    float sx = 1, sy = 1;
    glfwGetWindowContentScale(window, &sx, &sy);
    viewport = new cViewport(camera, sx, sy);
    return true;
  }

  void stop_haptics() {
    if (!thread) return;
    running = false;
    for (int i = 0; i < 300 && !finished; ++i) cSleepMs(10);
    if (!finished) std::fprintf(stderr, "[touch] haptic thread did not stop in 3 s\n");
    delete thread;
    thread = nullptr;
  }
};

HapticApp::HapticApp(Shared& shared, const Config& cfg, const PlanarArm& arm, const RobotModel& model)
    : impl_(std::make_unique<Impl>(shared, cfg, arm, model)) {}

HapticApp::~HapticApp() {
  Impl& m = *impl_;
  m.stop_haptics();                      // zero force, stop loop
  if (m.device_open) m.device->close();  // then close
  delete m.handler;
  delete m.world;
  if (m.window) {
    glfwDestroyWindow(m.window);
    glfwTerminate();
  }
}

bool HapticApp::open(std::string& info) {
  Impl& m = *impl_;
  m.handler = new cHapticDeviceHandler();
  if (m.handler->getNumDevices() == 0) {
    info = "no haptic device found (is the Touch plugged in, and Touch Smart Setup closed?)";
    return false;
  }
  m.handler->getDevice(m.device, 0);
  if (!m.device || !m.device->open()) {
    info = "could not open the haptic device";
    return false;
  }
  m.device_open = true;
  m.device->setForceAndTorqueAndGripperForce(kZero, kZero, 0.0);  // hdPhantom enables force output on open
  const cHapticDeviceInfo spec = m.device->getSpecifications();
  std::ostringstream ss;
  ss << "device: " << spec.m_modelName << " (" << spec.m_manufacturerName << ")  max force " << spec.m_maxLinearForce
     << " N, max stiffness " << spec.m_maxLinearStiffness << " N/m, max damping " << spec.m_maxLinearDamping << " N/(m/s)";
  info = ss.str();
  return true;
}

void HapticApp::run(const std::function<bool()>& keep_running) {
  Impl& m = *impl_;
  m.shared.wall_on = m.cfg.wall_enabled;
  m.shared.feedback_on = m.cfg.feedback_enabled;
  if (!m.open_window()) {
    std::fprintf(stderr, "[touch] could not open the window (OpenGL/GLFW)\n");
    return;
  }
  m.running = true;
  m.finished = false;
  m.thread = new cThread();
  m.thread->start(&Impl::trampoline, CTHREAD_PRIORITY_HAPTICS, &m);

  while (!glfwWindowShouldClose(m.window) && keep_running()) {
    m.update_scene();
    m.world->updateShadowMaps(false, false);
    m.viewport->renderView(m.fb_w, m.fb_h);
    glFinish();
    glfwSwapBuffers(m.window);
    glfwPollEvents();
    m.graphics_rate.signal(1);
  }
  m.stop_haptics();
}

}  // namespace touch
