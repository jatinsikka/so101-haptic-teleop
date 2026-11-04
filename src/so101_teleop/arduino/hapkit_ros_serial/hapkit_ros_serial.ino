// hapkit_ros_serial.ino -- Hapkit side of the link to the ROS `hapkit` node.
//
// Suggested protocol (115200 baud, ASCII lines):
//   Arduino -> PC   "P <x>\n"   handle position, ~200 Hz
//   PC -> Arduino   "F <f>\n"   force to render, N
//
// Start from your course Hapkit code (position from the MR sensor, force -> motor output).
// What to add:
//   1. Serial.begin(115200) in setup()
//   2. in loop(): read incoming bytes WITHOUT blocking (Serial.available()), collect them
//      into a line buffer, and when you see '\n' parse "F <f>" (atof) into forceCmd
//   3. clamp forceCmd to a safe maximum, here AND on the PC
//   4. watchdog: if no "F" line for ~200 ms, render 0 (the PC side died -> handle goes slack)
//   5. send "P <xh>" every ~5 ms using micros(), not delay(): delay() would stall your haptic loop
//
// Design question worth answering in your report: the ROS force arrives at ~50 Hz but your
// Arduino loop runs at several kHz. What should run locally on the Arduino (walls, damping)
// and what is fine to come over ROS?

void setup() {
  Serial.begin(115200);
  // TODO: your Hapkit setup
}

void loop() {
  // TODO: position update, serial in, watchdog, force output, serial out
}
