#!/usr/bin/env python3
"""
base_motion_server.py  –  ROS 2 node exposing seven motion primitives
Pi 5 • Ubuntu 24.04 • gpiozero  +  lgpio pin‑factory  (Pi‑5 compatible)
Keeps GAINS  *and* INVERT logic from omni_demo_calibrated.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from time import sleep

# ---------- gpiozero on Pi‑5 --------------------------------------------------
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Device
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()             # Pi‑5‑safe backend
# -----------------------------------------------------------------------------


# ─────────── Hardware map (BCM) ───────────
PINS = {
    "RR": (12,  5,  6),
    "RL": (13, 16, 20),
    "FR": (18, 21, 26),
    "FL": (19, 23, 24)
}

# ─────────── Calibration ───────────
GAINS  = {"RR": 0.55, "RL": 0.55, "FR": 0.45, "FL": 1.0}
INVERT = {"RR": True, "RL": True, "FR": True, "FL": False}   # same as test.py
BASE_SPEED   = 0.8
PWM_FREQ     = 1000
DEFAULT_TIME = 1.0


# ─────────── Low‑level wheel object ───────────
class Motor:
    def __init__(self, en, in1, in2, gain, invert):
        self.dir1 = DigitalOutputDevice(in1)
        self.dir2 = DigitalOutputDevice(in2)
        self.pwm  = PWMOutputDevice(en, frequency=PWM_FREQ)
        self.gain   = gain
        self.invert = invert

    def set(self, value):                     # value −1 … +1
        # gain + overall throttle
        v = max(-1, min(1, value * self.gain)) * BASE_SPEED
        fwd = v >= 0
        # XOR with wiring‑invert map
        if fwd ^ self.invert:
            self.dir1.on();  self.dir2.off()
        else:
            self.dir1.off(); self.dir2.on()
        self.pwm.value = abs(v)

    def stop(self):
        self.pwm.value = 0.0


# ─────────── ROS 2 node ───────────
class BaseMotion(Node):
    def __init__(self):
        super().__init__('base_motion_server')

        # create Motor objects
        self.motors = {
            name: Motor(*PINS[name], GAINS[name], INVERT[name])
            for name in PINS
        }

        # parameters
        self.declare_parameter('duration',   DEFAULT_TIME)
        self.declare_parameter('base_speed', BASE_SPEED)

        # services
        self._srv('move_forward',   lambda: self._go(fwd=+1))
        self._srv('move_backward',  lambda: self._go(fwd=-1))
        self._srv('strafe_left',    lambda: self._go(strafe=+1))
        self._srv('strafe_right',   lambda: self._go(strafe=-1))
        self._srv('turn_left',      lambda: self._go(spin=-1))
        self._srv('turn_right',     lambda: self._go(spin=+1))
        self._srv('turn_180',       lambda: self._go(spin=+1, extra=1.0))

    # -------- helpers --------
    def _srv(self, name, fn):
        self.create_service(Trigger, name,
            lambda req, resp: self._wrap(fn, resp))

    def _wrap(self, fn, resp):
        try:
            fn()
            resp.success, resp.message = True, 'OK'
        except Exception as e:
            resp.success, resp.message = False, str(e)
        return resp

    # mecanum mixing identical to test.py
    def _go(self, *, fwd=0, strafe=0, spin=0, extra=0.0):
        wheels = {
            "RR":  fwd - strafe - spin,
            "RL":  fwd + strafe + spin,
            "FR":  fwd + strafe - spin,
            "FL":  fwd - strafe + spin,
        }
        for w,v in wheels.items():
            self.motors[w].set(v)

        t = self.get_parameter('duration').value + extra
        sleep(t)

        for m in self.motors.values():
            m.stop()


def main():
    rclpy.init()
    node = BaseMotion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
