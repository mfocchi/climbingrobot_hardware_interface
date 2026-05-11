#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import matplotlib.pyplot as plt

import rospy

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

from climbingrobot_hardware_interface.msg import RopeCommand, RopeTelemetry


class PositionStepTest:

    def __init__(self):
        self.side             = rospy.get_param('~side',             'left')
        self.step_m           = float(rospy.get_param('~step_m',           0.30))
        self.hold_0_s         = float(rospy.get_param('~hold_0_s',         2.0))
        self.hold_step_s      = float(rospy.get_param('~hold_step_s',      18.0))
        self.hold_back_s      = float(rospy.get_param('~hold_back_s',      18.0))
        self.rate_hz          = float(rospy.get_param('~rate_hz',          50.0))
        self.do_rope_zero     = bool(rospy.get_param('~do_rope_zero',      True))
        self.disengage_brake  = bool(rospy.get_param('~disengage_brake',   True))
        self.output_csv       = str(rospy.get_param('~output_csv',         '/tmp/position_step_test.csv'))

        base = f"/winch/{self.side}"

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_mode = rospy.Publisher(f"{base}/set_motor_mode", String,      queue_size=10)
        self.pub_cmd  = rospy.Publisher(f"{base}/command",        RopeCommand, queue_size=10)

        # ── Subscriber ───────────────────────────────────────────────────
        self.sub_tel = rospy.Subscriber(
            f"{base}/telemetry",
            RopeTelemetry,
            self.telemetry_cb,
            queue_size=10,
        )

        # ── Service proxies ──────────────────────────────────────────────
        self.srv_rope_zero        = f"{base}/rope_zero"
        self.srv_brake_disengage  = f"{base}/brake_disengage"
        self.srv_brake_engage     = f"{base}/brake_engage"

        self.latest_tel = None
        self.samples    = []

        rospy.loginfo(
            f"Position step test ready on {base}: step_m={self.step_m:.3f} m"
        )

    # ────────────────────────────────────────────────────────────────────
    # Telemetry callback
    # ────────────────────────────────────────────────────────────────────

    def telemetry_cb(self, msg):
        self.latest_tel = msg

    # ────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────

    def wait_for_telemetry(self, timeout_s=5.0):
        rospy.loginfo("Waiting for telemetry...")
        t0 = time.time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.latest_tel is None:
            if time.time() - t0 > timeout_s:
                raise RuntimeError("No RopeTelemetry received")
            rate.sleep()
        rospy.loginfo("Telemetry received")

    def call_trigger(self, srv_name, timeout_s=5.0):
        rospy.loginfo(f"Waiting for service: {srv_name}")
        try:
            rospy.wait_for_service(srv_name, timeout=timeout_s)
        except rospy.ROSException:
            raise RuntimeError(f"Service not available: {srv_name}")

        proxy = rospy.ServiceProxy(srv_name, Trigger)
        resp = proxy(TriggerRequest())

        if not resp.success:
            raise RuntimeError(f"Service failed: {srv_name} — {resp.message}")

        rospy.loginfo(f"{srv_name}: {resp.message}")

    def set_mode(self, mode: str):
        msg = String(data=mode)
        for _ in range(10):
            self.pub_mode.publish(msg)
            rospy.sleep(0.02)
        rospy.loginfo(f"Mode sent: {mode}")

    def send_position(self, position_m: float):
        msg = RopeCommand()
        msg.header.stamp  = rospy.Time.now()
        msg.rope_force    = float('nan')
        msg.rope_velocity = float('nan')
        msg.rope_position = float(position_m)
        self.pub_cmd.publish(msg)

    # ────────────────────────────────────────────────────────────────────
    # Phase runner
    # ────────────────────────────────────────────────────────────────────

    def run_phase(self, ref_m: float, duration_s: float, t_start: float):
        period = 1.0 / max(self.rate_hz, 1.0)
        t_end  = time.time() + duration_s
        rate   = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown() and time.time() < t_end:
            now = time.time()
            self.send_position(ref_m)

            if self.latest_tel is not None:
                t        = now - t_start
                actual   = float(self.latest_tel.rope_length)
                velocity = float(self.latest_tel.rope_velocity)
                current  = float(self.latest_tel.current)
                brake    = bool(self.latest_tel.brake_status)
                error    = ref_m - actual

                self.samples.append({
                    'time_s':       t,
                    'reference_m':  ref_m,
                    'actual_m':     actual,
                    'error_m':      error,
                    'velocity_m_s': velocity,
                    'current_A':    current,
                    'brake_status': int(brake),
                })

            rate.sleep()

    # ────────────────────────────────────────────────────────────────────
    # Save / plot
    # ────────────────────────────────────────────────────────────────────

    def save_csv(self):
        path = Path(self.output_csv)
        path.parent.mkdir(parents=True, exist_ok=True)

        with path.open('w', newline='') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    'time_s',
                    'reference_m',
                    'actual_m',
                    'error_m',
                    'velocity_m_s',
                    'current_A',
                    'brake_status',
                ],
            )
            writer.writeheader()
            writer.writerows(self.samples)

        rospy.loginfo(f"Saved CSV: {path}")

    def plot(self):
        if not self.samples:
            rospy.logwarn("No samples to plot")
            return

        t   = [s['time_s']      for s in self.samples]
        ref = [s['reference_m'] for s in self.samples]
        act = [s['actual_m']    for s in self.samples]
        err = [s['error_m']     for s in self.samples]

        plt.figure()
        plt.plot(t, ref, label='reference rope_position [m]', linewidth=2)
        plt.plot(t, act, label='actual rope_length [m]',      linewidth=2)
        plt.xlabel('time [s]')
        plt.ylabel('rope length [m]')
        plt.title(f'Winch {self.side} position control step')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure()
        plt.plot(t, err, label='tracking error [m]', linewidth=2)
        plt.xlabel('time [s]')
        plt.ylabel('error [m]')
        plt.title(f'Winch {self.side} position tracking error')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    # ────────────────────────────────────────────────────────────────────
    # Main execution sequence
    # ────────────────────────────────────────────────────────────────────

    def execute(self):
        self.wait_for_telemetry()

        if self.disengage_brake:
            self.call_trigger(self.srv_brake_disengage)
            rospy.sleep(1.0)

        if self.do_rope_zero:
            self.call_trigger(self.srv_rope_zero)
            rospy.sleep(0.5)

        self.set_mode('closed_loop_position')
        rospy.sleep(0.5)

        t_start = time.time()

        rospy.loginfo("Phase 1: reference = 0.0 m")
        self.run_phase(0.0, self.hold_0_s, t_start)

        rospy.loginfo(f"Phase 2: step reference = {self.step_m:.3f} m")
        self.run_phase(self.step_m, self.hold_step_s, t_start)

        rospy.loginfo("Phase 3: back to reference = 0.0 m")
        self.run_phase(0.0, self.hold_back_s, t_start)

        self.send_position(0.0)
        rospy.sleep(0.2)

        self.save_csv()
        self.plot()

        rospy.loginfo("Position step test completed")


def main():
    rospy.init_node('position_step_test')
    node = PositionStepTest()

    try:
        node.execute()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == '__main__':
    main()