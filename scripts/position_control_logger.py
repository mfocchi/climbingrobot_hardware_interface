#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import matplotlib.pyplot as plt

import rospy

from climbingrobot_hardware_interface.msg import RopeCommand, RopeTelemetry


class PositionControlLogger:

    def __init__(self):
        self.side       = rospy.get_param('~side', 'left')
        self.output_csv = rospy.get_param('~output_csv', '/home/andrea/position_control_log.csv')

        base = f"/winch/{self.side}"

        self.sub_cmd = rospy.Subscriber(
            f"{base}/command",
            RopeCommand,
            self.command_callback,
            queue_size=10,
        )
        self.sub_tel = rospy.Subscriber(
            f"{base}/telemetry",
            RopeTelemetry,
            self.telemetry_callback,
            queue_size=10,
        )

        self.t0             = time.time()
        self.last_reference = float('nan')
        self.samples        = []

        rospy.loginfo(
            f"Logger ready. Listening to {base}/command and {base}/telemetry"
        )

    # ────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────

    def command_callback(self, msg):
        self.last_reference = float(msg.rope_position)
        rospy.loginfo(f"Reference received: rope_position = {self.last_reference:.4f} m")

    def telemetry_callback(self, msg):
        t      = time.time() - self.t0
        actual = float(msg.rope_length)
        error  = self.last_reference - actual

        self.samples.append({
            'time_s':           t,
            'reference_m':      self.last_reference,
            'actual_m':         actual,
            'error_m':          error,
            'rope_velocity_m_s': float(msg.rope_velocity),
            'rope_force_N':     float(msg.rope_force),
            'current_A':        float(msg.current),
            'brake_status':     int(bool(msg.brake_status)),
        })

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
                    'rope_velocity_m_s',
                    'rope_force_N',
                    'current_A',
                    'brake_status',
                ],
            )
            writer.writeheader()
            writer.writerows(self.samples)

        rospy.loginfo(f"Saved CSV: {path}")

    def plot(self):
        valid = [s for s in self.samples if s['reference_m'] == s['reference_m']]  # NaN filter

        if not valid:
            rospy.logwarn("No valid samples with reference received.")
            return

        t   = [s['time_s']      for s in valid]
        ref = [s['reference_m'] for s in valid]
        act = [s['actual_m']    for s in valid]
        err = [s['error_m']     for s in valid]

        plt.figure()
        plt.plot(t, ref, label='reference rope_position [m]', linewidth=2)
        plt.plot(t, act, label='actual rope_length [m]',      linewidth=2)
        plt.xlabel('time [s]')
        plt.ylabel('rope length [m]')
        plt.title(f'Winch {self.side} position control: actual vs reference')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure()
        plt.plot(t, err, label='tracking error [m]', linewidth=2)
        plt.xlabel('time [s]')
        plt.ylabel('error [m]')
        plt.title(f'Winch {self.side} position error')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


def main():
    rospy.init_node('position_control_logger')
    node = PositionControlLogger()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.plot()


if __name__ == '__main__':
    main()