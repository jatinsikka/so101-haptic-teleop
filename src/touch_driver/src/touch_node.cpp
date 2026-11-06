// ROS side of the Touch leader (no CHAI3D here). Params -> capped Config, URDF -> kinematics.
//   pub  /leader/joint_states  arm target, 5 joints, 100 Hz
//        /leader/engaged       deadman: teleop moves the follower only while true
//        /touch/force          [fx, fy, fz] sent to the Touch [N] (touch frame)
//                              (Float64MultiArray: Smart App Control blocks geometry_msgs' DLL)
//   sub  /follower/joint_states
//   srv  /touch/clear_fault
// Threads: main = window, ROS spin thread, CHAI3D haptic thread. Shared via shared.hpp.
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <thread>

#include "touch_driver/haptics.hpp"
#include "touch_driver/kinematics.hpp"
#include "touch_driver/shared.hpp"

using namespace std::chrono_literals;
using touch::Arm5;
using touch::Vec3;

static const char* kJoints[5] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};

class TouchRos : public rclcpp::Node {
 public:
  explicit TouchRos(touch::Shared& shared) : Node("touch_driver"), shared_(shared) {
    load_config();
    const std::string urdf = declare_parameter<std::string>("robot_description", "");
    std::string err = "robot_description is empty";
    if (urdf.empty() || !touch::planar_from_urdf(urdf, arm_, err)) throw std::runtime_error("SO-101 kinematics: " + err);
    if (!touch::robot_from_urdf(urdf, model_, err)) throw std::runtime_error("SO-101 model: " + err);

    pub_q_ = create_publisher<sensor_msgs::msg::JointState>("/leader/joint_states", 10);
    pub_engaged_ = create_publisher<std_msgs::msg::Bool>("/leader/engaged", 10);
    pub_force_ = create_publisher<std_msgs::msg::Float64MultiArray>("/touch/force", 10);
    sub_follower_ = create_subscription<sensor_msgs::msg::JointState>(
        "/follower/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { on_follower(*m); });
    srv_clear_ = create_service<std_srvs::srv::Trigger>(
        "/touch/clear_fault", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lk(shared_.from_ros_mtx);
          shared_.from_ros.clear_fault = true;
          res->success = true;
          res->message = "clear requested (takes effect while button 1 is released)";
        });
    timer_pub_ = create_wall_timer(10ms, [this] { publish(); });
    timer_log_ = create_wall_timer(2s, [this] { log_status(); });
  }

  const touch::Config& config() const { return cfg_; }
  const touch::PlanarArm& arm() const { return arm_; }
  const touch::RobotModel& model() const { return model_; }

 private:
  // declare + clamp to [lo, hi], warn if clamped
  double num(const std::string& name, double def, double lo, double hi) {
    const double v = declare_parameter(name, def);
    const double c = touch::clamp(v, lo, hi);
    if (c != v) RCLCPP_WARN(get_logger(), "%s = %g outside [%g, %g] -> using %g", name.c_str(), v, lo, hi, c);
    return c;
  }
  std::vector<double> list(const std::string& name, const std::vector<double>& def) {
    const auto v = declare_parameter(name, def);
    if (v.size() != def.size()) throw std::runtime_error(name + " must have " + std::to_string(def.size()) + " numbers");
    return v;
  }

  void load_config() {
    using namespace touch;
    Config& c = cfg_;
    c.enable_force = declare_parameter("enable_force", false);
    c.map.scale = num("map.scale", c.map.scale, 0.25, 5.0);
    const auto sg = list("map.axis_signs", {c.map.sign[0], c.map.sign[1], c.map.sign[2]});
    for (int i = 0; i < 3; ++i) {
      if (std::fabs(std::fabs(sg[i]) - 1.0) > 1e-9) throw std::runtime_error("map.axis_signs must be +1 or -1");
      c.map.sign[i] = sg[i];
    }
    c.max_joint_speed = num("max_joint_speed", c.max_joint_speed, 0.1, 4.0);
    c.stylus_axis = static_cast<int>(clamp(declare_parameter("map.stylus_axis", 0), 0, 2));
    c.pitch_gain = num("map.pitch_gain", c.pitch_gain, -2.0, 2.0);
    c.roll_gain = num("map.roll_gain", c.roll_gain, -2.0, 2.0);

    c.max_force_N = num("max_force_N", c.max_force_N, 0.0, kHardMaxForce_N);
    c.slew_N_per_s = num("slew_N_per_s", c.slew_N_per_s, 0.0, kHardMaxSlew_N_per_s);
    c.engage_ramp_s = num("engage_ramp_s", c.engage_ramp_s, 0.1, 5.0);
    c.free_damping = num("free_damping", c.free_damping, 0.0, kHardMaxDamping_Ns_per_m);
    c.velocity_lpf_hz = num("velocity_lpf_hz", c.velocity_lpf_hz, 5.0, 200.0);

    c.wall_enabled = declare_parameter("wall.enabled", c.wall_enabled);
    auto p = list("wall.point", {c.wall.point[0], c.wall.point[1], c.wall.point[2]});
    auto n = list("wall.normal", {c.wall.normal[0], c.wall.normal[1], c.wall.normal[2]});
    c.wall.point = {p[0], p[1], p[2]};
    const Vec3 nv{n[0], n[1], n[2]};
    if (norm(nv) < 1e-6) throw std::runtime_error("wall.normal must not be zero");
    c.wall.normal = mul(1.0 / norm(nv), nv);
    c.wall.stiffness = num("wall.stiffness", c.wall.stiffness, 0.0, kHardMaxStiffness_N_per_m);
    c.wall.damping = num("wall.damping", c.wall.damping, 0.0, kHardMaxDamping_Ns_per_m);

    c.reach_enabled = declare_parameter("reach.enabled", c.reach_enabled);
    c.reach_stiffness = num("reach.stiffness", c.reach_stiffness, 0.0, kHardMaxStiffness_N_per_m);

    c.feedback_enabled = declare_parameter("feedback.enabled", c.feedback_enabled);
    c.feedback_stiffness = num("feedback.stiffness", c.feedback_stiffness, 0.0, kHardMaxStiffness_N_per_m);
    c.feedback_deadband_m = num("feedback.deadband_m", c.feedback_deadband_m, 0.0, 0.1);
    c.feedback_lpf_hz = num("feedback.lpf_hz", c.feedback_lpf_hz, 1.0, 200.0);
    c.watchdog_s = num("feedback.watchdog_s", c.watchdog_s, 0.02, 1.0);

    const auto h = list("home", std::vector<double>(c.home.begin(), c.home.end()));
    for (int i = 0; i < 5; ++i) c.home[i] = h[i];
    c.description_dir = declare_parameter<std::string>("description_dir", "");

    RCLCPP_INFO(get_logger(), "forces %s | max %.2f N | scale %.2f, signs [%+.0f %+.0f %+.0f] | wall %s | feedback %s",
                c.enable_force ? "ENABLED" : "DISABLED (zero force only)", c.max_force_N, c.map.scale, c.map.sign[0],
                c.map.sign[1], c.map.sign[2], c.wall_enabled ? "on" : "off", c.feedback_enabled ? "on" : "off");
  }

  void on_follower(const sensor_msgs::msg::JointState& m) {
    Arm5 q{};
    int found = 0;
    for (int i = 0; i < 5; ++i)
      for (size_t k = 0; k < m.name.size() && k < m.position.size(); ++k)
        if (m.name[k] == kJoints[i] && std::isfinite(m.position[k])) {
          q[i] = m.position[k];
          ++found;
          break;
        }
    if (found != 5) return;  // incomplete: ignore
    std::lock_guard<std::mutex> lk(shared_.from_ros_mtx);
    shared_.from_ros.q_follower = q;
    shared_.from_ros.stamp = touch::Clock::now();
    shared_.from_ros.have_follower = true;
  }

  touch::HapticState snapshot() {
    std::lock_guard<std::mutex> lk(shared_.state_mtx);
    return shared_.state;
  }

  void publish() {
    const touch::HapticState s = snapshot();
    if (s.ticks == 0) return;  // loop not running yet

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    for (int i = 0; i < 5; ++i) {
      js.name.emplace_back(kJoints[i]);
      js.position.push_back(s.q_target[i]);
    }
    pub_q_->publish(js);

    std_msgs::msg::Bool b;
    b.data = s.engaged && !s.fault;
    pub_engaged_->publish(b);

    std_msgs::msg::Float64MultiArray f;
    f.data.assign(s.force.begin(), s.force.end());
    pub_force_->publish(f);
  }

  void log_status() {
    const touch::HapticState s = snapshot();
    if (s.ticks == 0) {
      RCLCPP_WARN(get_logger(), "haptic loop not running yet");
      return;
    }
    if (s.fault)
      RCLCPP_ERROR(get_logger(), "FAULT: %s (release button 1, then ros2 service call /touch/clear_fault std_srvs/srv/Trigger)",
                   touch::fault_name(s.fault_reason));
    else
      RCLCPP_INFO(get_logger(), "%s | %.0f Hz | |F| %.2f N | follower %s", s.engaged ? "ENGAGED" : "free", s.rate_hz,
                  touch::norm(s.force), s.follower_live ? "live" : "not heard");
    if (s.rate_hz > 0 && s.rate_hz < 800) RCLCPP_WARN(get_logger(), "haptic rate %.0f Hz is below ~1 kHz", s.rate_hz);
  }

  touch::Shared& shared_;
  touch::Config cfg_;
  touch::PlanarArm arm_;
  touch::RobotModel model_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_q_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_engaged_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_force_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_follower_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;
  rclcpp::TimerBase::SharedPtr timer_pub_, timer_log_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  touch::Shared shared;
  int code = 0;
  try {
    auto node = std::make_shared<TouchRos>(shared);
    touch::HapticApp app(shared, node->config(), node->arm(), node->model());
    std::string info;
    if (!app.open(info)) {
      RCLCPP_FATAL(node->get_logger(), "%s", info.c_str());
      code = 1;
    } else {
      RCLCPP_INFO(node->get_logger(), "%s", info.c_str());
      std::thread ros([node] { rclcpp::spin(node); });
      app.run([] { return rclcpp::ok(); });  // returns on window close or Ctrl+C
      rclcpp::shutdown();                    // stops spin
      ros.join();
    }
  } catch (const std::exception& e) {  // ~HapticApp already ran: zero force, loop stopped, device closed
    std::fprintf(stderr, "[touch_driver] %s\n", e.what());
    code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  std::printf("[touch_driver] Touch released.\n");
  return code;
}
