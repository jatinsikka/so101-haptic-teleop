"""Detect the gripper closing on an object, and keep a wall on the Hapkit handle while it does.

Contact starts when the gripper has nearly stopped while its goal is still well below its position.
It deliberately ignores load: after an impact, load bounces (96, 332, 152...), and requiring it to stay
high delayed the wall by 180-430 ms in logged grasps.

There are two paths. A fast grasp leaves a large gap between goal and position, because the stall lead
only allows 50 ticks once stopped, so a large gap can be trusted after 20 ms. A slow grasp never builds
that gap, so it uses a smaller one held for 40 ms.

The handle has usually passed the contact point by detection time. Rendering the spring from the true
contact point at once would be a step of several hundred PWM, felt as a kick. So the wall starts at the
handle and moves to the contact point at a limited speed: a small overshoot lands quickly, and a large
one gently.

Once in contact, it stays in contact until the handle moves back right of the wall, or the gripper
closes further, meaning the object moved. Goal error cannot end it: at the wall the handle stops
asking the gripper to close, and releasing on that would make the wall buzz on and off.

Two wall types are available. "spring" is the wall alone. "surface" adds a decaying sinusoid at the
moment of contact, as in baseline case 9. It signals contact at once, even before the spring builds.
"""

import collections

import serial

# Entering contact. Error is how many ticks the position is above the goal.
CONTACT_MAX_SPEED = 300       # ticks/s
CONTACT_MIN_ERROR = 40
CONTACT_DEBOUNCE_S = 0.04
FAST_CONTACT_MAX_SPEED = 600  # ticks/s
FAST_CONTACT_MIN_ERROR = 80
FAST_CONTACT_DEBOUNCE_S = 0.02

# Leaving contact.
RELEASE_HANDLE_MARGIN_CM = 0.3
RELEASE_CLOSING_TICKS = 30

# Commands. While landing the wall moves every tick; after that, sending every 20 ms leaves plenty of
# margin on the firmware's 250 ms watchdog.
WALL_LANDING_SPEED_CM_S = 6.0
WALL_SEND_PERIOD_S = 0.02
MAX_WALL_K = 300.0            # matches K_MAX_PWM_PER_CM in the firmware, which also enforces it

WALL_SPRING = "spring"
WALL_SURFACE = "surface"

# Impact speed is the fastest closing speed just before contact, since the gripper has already
# stopped by the time contact is detected.
IMPACT_WINDOW_S = 0.2

# Surface tuning. Baseline case 9 used 20 Hz and 0.25 s, which this now matches. Commands below about
# 100 PWM appear lost to friction (45 PWM could not be felt), so the amplitude and decay keep more
# of the cycles above that level. The floor makes slow grasps felt.
SURFACE_FREQ_HZ = 20.0
SURFACE_DECAY_S = 0.25
SURFACE_BASE_AMP = 250.0      # PWM, at zero impact speed
SURFACE_AMP_PER_CM_S = 10.0   # PWM per cm/s of impact speed, measured at the handle
SURFACE_MAX_AMP = 400.0       # PWM; the motor command itself is clamped to +/-400


def surface_amplitude(impact_cm_s, base=SURFACE_BASE_AMP, gain=SURFACE_AMP_PER_CM_S,
                      maximum=SURFACE_MAX_AMP):
    return min(maximum, base + gain * max(0.0, impact_cm_s))


class ContactWall:
    """Tracks contact and sends commands to the Hapkit.

    to_handle maps gripper ticks to xh in cm. ticks_per_cm converts gripper speed to handle speed.
    """

    def __init__(self, ser, to_handle, k, wall_type=WALL_SPRING, ticks_per_cm=112.6,
                 surface_base=SURFACE_BASE_AMP, surface_gain=SURFACE_AMP_PER_CM_S,
                 landing_speed=WALL_LANDING_SPEED_CM_S, surface_max=SURFACE_MAX_AMP):
        self.ser = ser
        self.to_handle = to_handle
        self.k = min(k, MAX_WALL_K)
        self.wall_type = wall_type
        self.ticks_per_cm = ticks_per_cm
        self.surface_base = surface_base
        self.surface_gain = surface_gain
        self.surface_max = surface_max
        self.landing_speed = landing_speed
        self.speeds = collections.deque()
        self.impact_amp = None
        self._clear()

    def configure_surface(self, freq_hz=SURFACE_FREQ_HZ, decay_s=SURFACE_DECAY_S):
        self._send(f"C {freq_hz:.1f} {decay_s:.3f}\n")

    def update(self, now, xh, position, goal, speed):
        """Advance one tick. Returns "contact", "released" or None when nothing changed."""
        self.speeds.append((now, speed))
        while self.speeds and now - self.speeds[0][0] > IMPACT_WINDOW_S:
            self.speeds.popleft()

        event = None
        if self.in_contact:
            if (xh > self.x_c + RELEASE_HANDLE_MARGIN_CM
                    or position < self.contact_position - RELEASE_CLOSING_TICKS):
                self.stop()
                event = "released"
        elif self._detected(now, position, goal, speed):
            self.in_contact = True
            self.contact_position = position
            self.contact_time = now
            self.next_send = now
            self.x_c = self.to_handle(position)
            self.landing_start = min(xh, self.x_c)
            self.commanded_k = self.k
            event = "contact"

        if self.in_contact:
            wall_position = self._wall_position(now)
            if wall_position < self.x_c or now >= self.next_send:
                self.next_send = now + WALL_SEND_PERIOD_S
                self._send(f"W {wall_position:.2f} {self.commanded_k:.1f}\n")

        if event == "contact" and self.wall_type == WALL_SURFACE:
            # After the first W: the firmware ignores I when no wall is active.
            self.impact_amp = surface_amplitude(self.impact_speed(), self.surface_base,
                                                self.surface_gain, self.surface_max)
            self._send(f"I {self.impact_amp:.0f}\n")
        return event

    def impact_speed(self):
        """The fastest recent closing speed, as handle speed in cm/s."""
        closing = max((-speed for _time, speed in self.speeds), default=0)
        return max(0.0, closing) / self.ticks_per_cm

    def _wall_position(self, now):
        if self.landing_speed <= 0:
            return self.x_c
        moved = self.landing_speed * (now - self.contact_time)
        return min(self.x_c, self.landing_start + moved)

    def _detected(self, now, position, goal, speed):
        error = position - goal
        abs_speed = abs(speed)

        if abs_speed <= CONTACT_MAX_SPEED and error >= CONTACT_MIN_ERROR:
            if self.slow_since is None:
                self.slow_since = now
        else:
            self.slow_since = None

        if abs_speed <= FAST_CONTACT_MAX_SPEED and error >= FAST_CONTACT_MIN_ERROR:
            if self.fast_since is None:
                self.fast_since = now
        else:
            self.fast_since = None

        return ((self.slow_since is not None and now - self.slow_since >= CONTACT_DEBOUNCE_S)
                or (self.fast_since is not None
                    and now - self.fast_since >= FAST_CONTACT_DEBOUNCE_S))

    def stop(self):
        """Turn the wall off. Always sends R, so this is safe to call at any time."""
        self._clear()
        self._send("R\n")

    def _clear(self):
        self.in_contact = False
        self.slow_since = None
        self.fast_since = None
        self.contact_position = None
        self.contact_time = None
        self.next_send = None
        self.x_c = None
        self.landing_start = None
        self.commanded_k = 0.0

    def _send(self, text):
        # A dropped W is harmless because the next one follows shortly. If they stop, the firmware's
        # watchdog removes the wall.
        try:
            self.ser.write(text.encode())
        except serial.SerialTimeoutException:
            pass
