// Tests for haptic_math.hpp. No device, no ROS. Exit code 0 = all passed.
// Each test is also a small, checkable statement of a concept from the README.
#include <cstdio>
#include <random>

#include "touch_driver/haptic_math.hpp"

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
static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
static bool near3(const Vec3& a, const Vec3& b, double tol = 1e-9) { return norm(sub(a, b)) <= tol; }

// The clutch: at the anchor the arm is exactly at its anchor. Default signs [1,-1,1], scale 2:
// stylus 1 cm TOWARD you (+x touch) -> arm 2 cm OUT (+x arm); 1 cm to your right -> arm 2 cm to -y.
static void test_workspace_map() {
  WorkspaceMap m;
  m.touch_anchor = {0.01, -0.02, 0.03};
  m.arm_anchor = {0.25, 0.0, 0.15};
  CHECK(near3(touch_to_arm(m, m.touch_anchor), m.arm_anchor), "anchor");
  const Vec3 p = touch_to_arm(m, add(m.touch_anchor, {0.01, 0, 0}));
  CHECK(near3(p, {0.27, 0.0, 0.15}), "toward you -> arm out: %g %g %g", p[0], p[1], p[2]);
  const Vec3 rt = touch_to_arm(m, add(m.touch_anchor, {0, 0.01, 0}));
  CHECK(near3(rt, {0.25, -0.02, 0.15}), "your right -> arm -y");
  const Vec3 up = touch_to_arm(m, add(m.touch_anchor, {0, 0, 0.01}));
  CHECK(near3(up, {0.25, 0.0, 0.17}), "up -> up");
  // a spring in the arm frame, mapped back, has the SAME direction as the hand motion that caused it
  const Vec3 d_touch{0.003, -0.004, 0.002};
  const Vec3 d_arm = sub(touch_to_arm(m, add(m.touch_anchor, d_touch)), m.arm_anchor);
  CHECK(near3(arm_delta_to_touch(m, d_arm), d_touch), "round trip");
}

// Twist: rotate a stylus frame about its own axis by a known angle; we must measure that angle.
static void test_twist() {
  const Vec3 a{0, 0.6, 0.8};   // some stylus axis (unit)
  const Vec3 b0{1, 0, 0};      // perpendicular to it
  const Vec3 c = cross(a, b0); // the third axis
  for (double ang : {0.0, 0.3, -1.2, 2.5}) {
    const Vec3 b = add(mul(std::cos(ang), b0), mul(std::sin(ang), c));
    CHECK(near(twist_about(a, b0, b), ang, 1e-12), "twist %g got %g", ang, twist_about(a, b0, b));
  }
  CHECK(near(stylus_pitch({0, 0, 1}), kPi / 2), "pitch up");
  CHECK(near(stylus_pitch({1, 0, 0}), 0.0), "pitch level");
}

static void test_wall() {
  Wall w;  // plane z = 0.05, normal +z
  WallProxy r = wall_proxy(w, {0.3, 0, 0.10});
  CHECK(r.depth < 0 && near3(r.proxy, {0.3, 0, 0.10}), "free space: proxy = hand");
  r = wall_proxy(w, {0.3, 0.1, 0.03});  // 2 cm inside
  CHECK(near(r.depth, 0.02, 1e-12) && near3(r.proxy, {0.3, 0.1, 0.05}, 1e-12), "proxy on the surface");
  CHECK(near(wall_force_mag(150, 0.5, 0.01, 0.0), 1.5, 1e-12), "spring");
  CHECK(near(wall_force_mag(150, 0.5, 0.01, -0.2), 1.6, 1e-12), "moving in: damper adds");
  CHECK(near(wall_force_mag(150, 0.5, 0.01, 10.0), 0.0), "leaving fast: never pulls");
  CHECK(near(wall_force_mag(150, 0.5, -0.01, 0.0), 0.0), "outside: nothing");
}

static void test_safe_force() {
  Vec3 out;
  const double dt = 0.001;
  // slew: from 0, a huge request moves only slew*dt per tick
  CHECK(safe_force({100, 0, 0}, {0, 0, 0}, 1.0, 10.0, dt, out), "finite input rejected");
  CHECK(near(norm(out), 0.01), "slew %g", norm(out));
  // clamp preserves direction
  Vec3 prev{0, 0, 0};
  for (int k = 0; k < 2000; ++k) { safe_force({3, 4, 0}, prev, 1.0, 10.0, dt, out); prev = out; }
  CHECK(near3(out, {0.6, 0.8, 0}, 1e-9), "clamp %g %g %g", out[0], out[1], out[2]);
  // compiled-in hard caps beat a reckless config
  safe_force({100, 0, 0}, {0, 0, 0}, 1e6, 1e6, dt, out);
  CHECK(norm(out) <= kHardMaxSlew_N_per_s * dt + 1e-12, "slew cap %g", norm(out));
  prev = {0, 0, 0};
  for (int k = 0; k < 2000; ++k) { safe_force({100, 0, 0}, prev, 1e6, 1e6, dt, out); prev = out; }
  CHECK(near(norm(out), kHardMaxForce_N), "hard cap %g", norm(out));
  // a stalled tick (dt = 0.5 s) must not allow a big jump
  safe_force({1, 0, 0}, {0, 0, 0}, 1.0, 10.0, 0.5, out);
  CHECK(norm(out) <= 10.0 * 0.002 + 1e-12, "dt cap %g", norm(out));
  // NaN anywhere -> refuse, output zero
  CHECK(!safe_force({std::nan(""), 0, 0}, {0.5, 0, 0}, 1.0, 10.0, dt, out), "NaN accepted");
  CHECK(near3(out, {0, 0, 0}), "NaN output not zero");
  CHECK(!safe_force({0, 0, 0}, {INFINITY, 0, 0}, 1.0, 10.0, dt, out), "inf prev accepted");
}

static void test_transient() {
  Transient c;  // gain 4, 100 Hz, 20 ms, max 0.4
  CHECK(near(transient_amplitude(c, 0.05), 0.2), "amplitude ~ impact speed");
  CHECK(near(transient_amplitude(c, 10.0), 0.4), "amplitude capped by max_N");
  CHECK(near(transient_amplitude(c, -1.0), 0.0), "leaving the wall: no tap");
  c.max_N = 100.0;
  CHECK(near(transient_amplitude(c, 10.0), kHardMaxTransient_N), "hard cap beats config");
  c.max_N = 0.4;
  double peak = 0;
  for (double t = 0; t < 0.2; t += 0.001) peak = std::max(peak, std::fabs(transient_value(c, 0.4, t)));
  CHECK(peak <= 0.4 && peak > 0.2, "peak %g", peak);
  CHECK(near(transient_value(c, 0.4, 0.0), 0.0), "starts at zero (no step)");
  CHECK(near(transient_value(c, 0.4, 0.15), 0.0) && near(transient_value(c, 0.4, -0.01), 0.0), "zero outside window");
  c.decay_s = 1.0;  // a long decay is still cut at the hard length
  CHECK(near(transient_value(c, 0.4, kHardMaxTransient_s + 0.001), 0.0), "hard length");
}

static void test_cbf() {
  Wall w;  // plane z = 0.05, normal +z
  const double alpha = 10.0, dt = 0.001;
  // far away, normal speed (0.1 mm per 1 ms tick = 0.1 m/s; limit there 10 x 0.25 = 2.5 m/s): follows the hand
  Vec3 s = cbf_step(w, alpha, {0.3, 0, 0.30}, {0.3, 0, 0.2999}, dt);
  CHECK(near3(s, {0.3, 0, 0.2999}), "far: follow the hand");
  // slam the hand 10 cm below the wall for 2 s: the safe point never crosses, slows, gets close
  s = {0.3, 0, 0.10};
  double worst_rate = 0;
  for (int k = 0; k < 2000; ++k) {
    const Vec3 next = cbf_step(w, alpha, s, {0.3, 0.001 * k, -0.05}, dt);  // also sliding sideways
    const double h = s[2] - 0.05, h_next = next[2] - 0.05;
    CHECK(h_next >= -1e-12, "crossed the wall at step %d (h %g)", k, h_next);
    worst_rate = std::max(worst_rate, (h - h_next) / dt - alpha * h);  // must be <= 0
    s = next;
  }
  CHECK(worst_rate <= 1e-9, "approach faster than alpha*h by %g m/s", worst_rate);
  CHECK(s[2] - 0.05 < 0.001, "should end close to the wall (h %g)", s[2] - 0.05);
  CHECK(near(s[1], 0.001 * 1999), "sideways motion untouched (%g)", s[1]);
  // moving away is never limited
  s = cbf_step(w, alpha, {0.3, 0, 0.051}, {0.3, 0, 0.20}, dt);
  CHECK(near(s[2], 0.20), "leaving is free");
}

int main() {
  test_workspace_map();
  test_twist();
  test_wall();
  test_safe_force();
  test_transient();
  test_cbf();
  std::printf(failures ? "%d FAILURE(S)\n" : "all haptic_math tests passed\n", failures);
  return failures ? 1 : 0;
}
