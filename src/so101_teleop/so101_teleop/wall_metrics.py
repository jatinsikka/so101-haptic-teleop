"""wall_metrics -- measure how the REAL arm meets the virtual wall: OFF vs IMPEDANCE vs CBF.

    ros2 run so101_teleop wall_metrics                         # while touch_teleop runs; Ctrl+C = summary
    ros2 run so101_teleop wall_metrics --ros-args -p table_gap_mm:=15.0

Reads /touch/metrics (from touch_driver, 50 Hz):
  [t, mode 0 OFF/1 IMPEDANCE/2 CBF, engaged, cmd tip h, real tip h, |F|, K, alpha, CBF filter on]
  h = tool-tip height above the wall in metres (< 0 = below the wall). engaged = button 1 held.

The real arm SAGS under gravity (it holds ~1 cm below its command). So "how far below the wall"
mixes two things. Per approach we report them separately:
  overshoot_mm   dynamic: how far the real tip dipped below where it then SETTLED
  sag_mm         static: settled real height - commanded height
  cmd_min_mm     lowest commanded height (>= 0: the command never entered the wall)
  below_wall_mm  lowest real height below the wall (overshoot + sag together)
  table_hit      below_wall_mm > table_gap_mm (table_gap_mm = the wall's height above the table)
  speed_mm_s     fastest downward speed of the real tip in the last 10 mm before the wall
  peak_force_N   strongest force you felt
One approach = engaged and the real tip within zone_mm of the wall, until it leaves (or you release).
The summary has one row per setting: OFF, IMPEDANCE, and CBF per (K, alpha). "+F" = filter always on.
Writes two CSVs in the current folder: every sample, and one row per approach.
"""

import csv
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

MODES = ["OFF", "IMPEDANCE", "CBF"]


class WallMetrics(Node):
    def __init__(self):
        super().__init__("wall_metrics")
        self.table_gap = float(self.declare_parameter("table_gap_mm", 15.0).value) / 1000
        self.zone = float(self.declare_parameter("zone_mm", 20.0).value) / 1000

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.samples_path = Path.cwd() / f"wall_samples_{stamp}.csv"
        self.approaches_path = Path.cwd() / f"wall_approaches_{stamp}.csv"
        self.samples_file = open(self.samples_path, "w", newline="")
        self.samples = csv.writer(self.samples_file)
        self.samples.writerow(["t", "setting", "engaged", "cmd_h_mm", "real_h_mm", "force_N"])

        self.results = []        # finished approaches
        self.cur = None          # approach in progress (dict) or None
        self.prev = None         # previous sample (t, real_h), for speed
        self.n_samples = self.n_engaged = 0
        self.create_subscription(Float64MultiArray, "/touch/metrics", self.on_sample, 100)
        self.create_timer(5.0, self.heartbeat)
        self.get_logger().info(f"logging to {self.samples_path.name}. Table {self.table_gap * 1000:.1f} mm "
                               f"below the wall. Ctrl+C for the summary.")

    def heartbeat(self):  # proof that data arrives and button 1 is seen
        self.get_logger().info(f"{self.n_samples} samples, {self.n_engaged} engaged, {len(self.results)} approaches")

    @staticmethod
    def setting(mode, K, alpha, filter_on):
        """'OFF', 'IMPEDANCE', 'CBF K10 a3.3 (K/a 3.0)', or '... +F' when the filter is always on."""
        name = MODES[mode]
        if mode == 2:
            return f"CBF K{K:.0f} a{alpha:.1f} (K/a {K / alpha:.1f})"
        return name + (" +F" if filter_on else "")

    def on_sample(self, msg):
        if len(msg.data) < 9:
            return
        t, mode, engaged, cmd_h, real_h, force, K, alpha, filt = msg.data[:9]
        setting = self.setting(int(round(mode)), K, alpha, filt > 0.5)
        self.n_samples += 1
        self.n_engaged += engaged > 0.5
        self.samples.writerow([f"{t:.3f}", setting, int(engaged), f"{cmd_h * 1000:.2f}", f"{real_h * 1000:.2f}",
                               f"{force:.3f}"])

        # downward speed of the real tip (m/s, > 0 = toward the wall)
        speed = 0.0
        if self.prev and t > self.prev[0]:
            speed = (self.prev[1] - real_h) / (t - self.prev[0])
        self.prev = (t, real_h)

        near = engaged > 0.5 and real_h < self.zone
        if near and self.cur is None:                      # 1. an approach starts
            self.cur = {"setting": setting, "order": (int(round(mode)), K / alpha), "t0": t, "min_h": real_h,
                        "i_min": 0, "cmd_min": cmd_h, "speed": 0.0, "force": 0.0, "on_wall": []}
        if self.cur is not None:
            c = self.cur                                   # 2. update it
            if real_h < c["min_h"]:
                c["min_h"], c["i_min"] = real_h, len(c["on_wall"])
            c["cmd_min"] = min(c["cmd_min"], cmd_h)
            c["force"] = max(c["force"], force)
            if 0.0 < real_h < 0.010:
                c["speed"] = max(c["speed"], speed)
            if cmd_h < 0.001:                              # target at (or below) the wall
                c["on_wall"].append((cmd_h, real_h))
            if not near:                                   # 3. it ends: score it
                self.finish(c, t)
                self.cur = None

    def finish(self, c, t):
        if c["min_h"] > 0.005:  # never got within 5 mm of the wall: not a contact attempt
            return
        after = c["on_wall"][c["i_min"]:]   # where it settled after the dip
        if len(after) >= 5:
            settled = statistics.median(r for _, r in after)
            sag = settled - statistics.median(cmd for cmd, _ in after)
        else:                               # touched and left at once: no settled value
            settled, sag = c["min_h"], float("nan")
        r = {"setting": c["setting"],
             "overshoot_mm": max(0.0, settled - c["min_h"]) * 1000,
             "sag_mm": sag * 1000,
             "cmd_min_mm": c["cmd_min"] * 1000,
             "below_wall_mm": max(0.0, -c["min_h"]) * 1000,
             "table_hit": c["min_h"] < -self.table_gap,
             "speed_mm_s": c["speed"] * 1000,
             "peak_force_N": c["force"],
             "duration_s": t - c["t0"],
             "_order": c["order"]}
        self.results.append(r)
        self.get_logger().info(
            f"#{len(self.results)} {r['setting']:<28} overshoot {r['overshoot_mm']:4.1f} mm  sag {r['sag_mm']:+5.1f} mm  "
            f"cmd min {r['cmd_min_mm']:+6.1f} mm  {'TABLE HIT' if r['table_hit'] else 'no hit   '}  "
            f"speed {r['speed_mm_s']:4.0f} mm/s")

    def summary(self):
        self.samples_file.close()
        rows = [{k: v for k, v in r.items() if k != "_order"} for r in self.results]
        if rows:
            with open(self.approaches_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
        print(f"\n {self.n_samples} samples, {self.n_engaged} engaged (button 1 held)")
        print(" setting                       n  overshoot mean/max  sag   cmd min  table hits  speed  force")
        print("                                  (mm, dynamic)       (mm)  (mm)                (mm/s) (N)")
        order = {r["setting"]: r["_order"] for r in self.results}
        for g in sorted(order, key=order.get):
            rs = [r for r in self.results if r["setting"] == g]

            def mean(k):  # skip NaN
                vals = [r[k] for r in rs if r[k] == r[k]]
                return statistics.fmean(vals) if vals else float("nan")

            print(f" {g:<28} {len(rs):2d}  {mean('overshoot_mm'):5.1f} / {max(r['overshoot_mm'] for r in rs):4.1f}"
                  f"       {mean('sag_mm'):+5.1f} {min(r['cmd_min_mm'] for r in rs):+6.1f}"
                  f"     {sum(r['table_hit'] for r in rs):2d}/{len(rs):<2d}     {mean('speed_mm_s'):5.0f}"
                  f"  {mean('peak_force_N'):.2f}")
        print(f"\n samples:    {self.samples_path}\n approaches: {self.approaches_path if rows else '(none)'}")


def main():
    rclpy.init()
    node = WallMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
