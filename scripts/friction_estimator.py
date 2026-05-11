#!/usr/bin/env python3
"""
friction_estimator.py — ROS 1 node for friction parameter estimation.

Steps:
  1. Apply a torque ramp sequence via /arganello/dx/target_torque
  2. Log telemetry from /arganello/dx/telemetry/enhanced to CSV
  3. Estimate friction parameters via linear regression (Striebeck model)
  4. Plot measured vs. model data
"""

import csv
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rospy

from std_msgs.msg import Float32
from climbingrobot_hardware_interface.msg import RopeTelemetry  # replace with ArganelloEnhancedTelemetry if available in ROS 1


# ─────────────────────────────────────────────────────────────
# Represents a single torque step: (torque in Nm, duration in seconds)
# ─────────────────────────────────────────────────────────────
class TorqueStep:
    def __init__(self, torque: float, duration: float):
        self.torque = torque
        self.duration = duration


# ─────────────────────────────────────────────────────────────
# Main ROS 1 Node for Friction Parameter Estimation
# ─────────────────────────────────────────────────────────────
class FrictionEstimatorNode:

    def __init__(self):
        # === Setup Output File ===
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'friction_log_{now}.csv'
        self.csv_header = "timestamp,motor_vel_rad_s,motor_torque\n"
        self.logging_active = False

        # === Define Torque Sequence ===
        self.sequence = [
            # torque (Nm)  duration (s)
            TorqueStep(0.6,   3.0),
            TorqueStep(+1.0,  0.8),
            TorqueStep(0.4,   3.0),
            TorqueStep(-0.1,  0.58),
            TorqueStep(0.6,   3.0),

            TorqueStep(0.6,   3.0),
            TorqueStep(+1.5,  0.315),
            TorqueStep(0.4,   3.0),
            TorqueStep(-0.1,  0.58),
            TorqueStep(0.6,   3.0),

            TorqueStep(0.6,   3.0),
            TorqueStep(+2.0,  0.21),
            TorqueStep(0.4,   3.0),
            TorqueStep(-0.1,  0.58),
            TorqueStep(0.6,   3.0),
        ]
        self.current_step_index = -1
        self.step_start_time = rospy.Time.now()

        # === ROS Communication ===
        self.publisher = rospy.Publisher('/arganello/dx/target_torque', Float32, queue_size=10)

        # NOTE: Replace RopeTelemetry with your actual ROS 1 enhanced telemetry message type
        # if you have ArganelloEnhancedTelemetry ported to ROS 1.
        # For now this uses RopeTelemetry as a placeholder — update the type and field names below.
        rospy.Subscriber(
            '/arganello/dx/telemetry/enhanced',
            RopeTelemetry,
            self.telemetry_callback,
            queue_size=10
        )

        # === Start Logging ===
        rospy.loginfo("═══════════════════════════════════════")
        rospy.loginfo("📌 STEP 1: Applying torque ramp...")
        rospy.loginfo("📌 STEP 2: Logging telemetry to CSV...")
        rospy.loginfo("═══════════════════════════════════════")
        self.file = open(self.filename, 'w')
        self.file.write(self.csv_header)
        self.logging_active = True

        # === Polling Timer (100 Hz) ===
        self.timer = rospy.Timer(rospy.Duration(0.01), self.step1_ramp_callback)

    # ─────────────────────────────────────────────────────────────
    # STEP 1: Torque Ramp Callback
    # ─────────────────────────────────────────────────────────────
    def step1_ramp_callback(self, event):
        now = rospy.Time.now()
        elapsed = (now - self.step_start_time).to_sec()

        if self.current_step_index == -1 or elapsed >= self.sequence[self.current_step_index].duration:
            self.current_step_index += 1

            if self.current_step_index >= len(self.sequence):
                self.publisher.publish(Float32(data=0.6))
                self.logging_active = False
                self.timer.shutdown()
                if not self.file.closed:
                    self.file.close()
                rospy.loginfo("✅ STEP 1 Complete: Torque ramp ended.")
                rospy.loginfo("✅ STEP 2 Complete: CSV logging ended.")
                rospy.loginfo(f"📂 Data saved to: {self.filename}")
                rospy.loginfo("═══════════════════════════════════════")
                rospy.loginfo("📌 STEP 3: Reloading CSV & estimating parameters...")
                rospy.loginfo("═══════════════════════════════════════")
                self.step3_estimate_parameters()
                return

            step = self.sequence[self.current_step_index]
            self.publisher.publish(Float32(data=step.torque))
            self.step_start_time = now
            rospy.loginfo(
                f"[Step {self.current_step_index + 1}/{len(self.sequence)}] "
                f"τ = {step.torque:.2f} Nm for {step.duration:.2f}s"
            )

    # ─────────────────────────────────────────────────────────────
    # STEP 2: Telemetry Callback
    # NOTE: Update field names below to match your actual ROS 1 message.
    # ROS 2 original used: msg.header.stamp.sec, msg.header.stamp.nanosec,
    #                      msg.motor_vel, msg.motor_torque
    # In ROS 1, header.stamp is a rospy.Time, so use .secs and .nsecs
    # ─────────────────────────────────────────────────────────────
    def telemetry_callback(self, msg):
        if not self.logging_active:
            return
        try:
            # ROS 1: header.stamp.secs + header.stamp.nsecs * 1e-9
            timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

            # TODO: replace with actual field names from your ROS 1 telemetry message
            motor_vel = msg.motor_vel * 2 * np.pi   # rev/s -> rad/s
            torque    = msg.motor_torque

            self.file.write(f"{timestamp:.6f},{motor_vel:.6f},{torque:.6f}\n")
        except Exception as e:
            rospy.logwarn(f"⚠️ Telemetry logging failed: {e}")

    # ─────────────────────────────────────────────────────────────
    # STEP 3: Load CSV and estimate friction parameters
    # ─────────────────────────────────────────────────────────────
    def step3_estimate_parameters(self):
        try:
            rospy.loginfo("📌 STEP 3: Estimating friction parameters via linear regression...")

            timestamps, velocities, torques = [], [], []
            with open(self.filename, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # skip header
                for row in reader:
                    ts, vel, tq = map(float, row)
                    timestamps.append(ts)
                    velocities.append(vel)
                    torques.append(tq)

            vel_arr    = np.array(velocities)
            torque_arr = np.array(torques)

            # Physical constants
            I            = 0.0    # Inertia [kg·m²]
            theta_ddot   = 0.0    # Angular acceleration [rad/s²]
            mass         = 1.55   # kg
            g            = 9.81   # m/s²
            r            = 0.020 + 0.0125  # effective radius [m]
            tau_gravity  = mass * g * r

            tau_friction = torque_arr - I * theta_ddot - tau_gravity

            # Search for best Striebeck threshold P3
            theta_th_values = np.linspace(0.1, 3.0, 200)
            best_error  = float('inf')
            best_params = None
            best_theta_th = None

            timestamp_str  = datetime.now().strftime("%Y%m%d_%H%M%S")
            debug_filename = f"debug_regression_{timestamp_str}.csv"

            with open(debug_filename, "w") as dbg:
                dbg.write("theta_th,P1,P2,P4,error\n")

                for theta_th in theta_th_values:
                    sgn = np.sign(vel_arr)
                    Ei  = np.exp(-np.abs(vel_arr) / theta_th)

                    A = np.column_stack((
                        sgn,        # P1: Coulomb friction
                        sgn * Ei,   # P2: Stiction exponential decay
                        vel_arr     # P4: Viscous friction
                    ))

                    y = tau_friction
                    x, *_ = np.linalg.lstsq(A, y, rcond=None)
                    residual = y - A @ x
                    error = np.linalg.norm(residual)

                    dbg.write(f"{theta_th:.6f},{x[0]:.6f},{x[1]:.6f},{x[2]:.6f},{error:.6f}\n")

                    if error < best_error:
                        best_error    = error
                        best_params   = x
                        best_theta_th = theta_th

            P1, P2, P4 = best_params
            P3 = best_theta_th

            rospy.loginfo("✅ STEP 3 Complete: Model fitted.")
            rospy.loginfo(f"   P1 = {P1:.4f} Nm (Coulomb friction)")
            rospy.loginfo(f"   P2 = {P2:.4f} Nm (Stiction component)")
            rospy.loginfo(f"   P3 = {P3:.4f} rad/s (Striebeck threshold)")
            rospy.loginfo(f"   P4 = {P4:.4f} Nm⋅s/rad (Viscous coefficient)")
            rospy.loginfo(f"📄 Regression debug saved to: {debug_filename}")

            self.step4_plot_results(vel_arr, torque_arr, P1, P2, P3, P4)

        except Exception as e:
            rospy.logerr(f"🚫 STEP 3 Failed: {e}")

    # ─────────────────────────────────────────────────────────────
    # STEP 4: Plot measured vs. model data
    # ─────────────────────────────────────────────────────────────
    def step4_plot_results(self, vel_arr, torque_arr, P1, P2, P3, P4):
        try:
            rospy.loginfo("═══════════════════════════════════════")
            rospy.loginfo("📌 STEP 4: Plotting model vs measured data...")
            rospy.loginfo(f"     • P1 = {P1:.4f} Nm (Coulomb friction)")
            rospy.loginfo(f"     • P2 = {P2:.4f} Nm (Stiction magnitude)")
            rospy.loginfo(f"     • P3 = {P3:.4f} rad/s (Striebeck threshold)")
            rospy.loginfo(f"     • P4 = {P4:.4f} Nm⋅s/rad (Viscous coefficient)")
            rospy.loginfo("═══════════════════════════════════════")

            predicted = (
                np.sign(vel_arr) * (P1 + P2 * np.exp(-np.abs(vel_arr) / P3))
                + P4 * vel_arr
            )

            v_dense     = np.linspace(min(vel_arr), max(vel_arr), 1000)
            model_dense = (
                np.sign(v_dense) * (P1 + P2 * np.exp(-np.abs(v_dense) / P3))
                + P4 * v_dense
            )

            # ── Plot 1: Friction Model Fit ──────────────────────────────
            plt.figure(figsize=(10, 6))
            plt.plot(vel_arr, torque_arr, 'b.', label='Measured Torque (Nm)', alpha=0.5)
            plt.plot(v_dense, model_dense, 'r-', linewidth=2, label='Fitted Friction Model')
            plt.xlabel("Angular Velocity $\\dot{\\theta}$ [rad/s]")
            plt.ylabel("Torque $\\tau$ [Nm]")
            plt.title(
                "Friction Model Fit\n"
                f"P1={P1:.3f} Nm, P2={P2:.3f} Nm, P3={P3:.3f} rad/s, P4={P4:.4f} Nm⋅s/rad"
            )
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()

            # ── Plot 2: Residuals ───────────────────────────────────────
            residuals = torque_arr - predicted
            plt.figure(figsize=(10, 4))
            plt.plot(vel_arr, residuals, 'k.', alpha=0.5)
            plt.axhline(0, color='gray', linestyle='--', linewidth=1)
            plt.xlabel("Angular Velocity $\\dot{\\theta}$ [rad/s]")
            plt.ylabel("Residual Torque $\\tau_{measured} - \\tau_{model}$ [Nm]")
            plt.title("Model Residuals (Torque Error vs Velocity)")
            plt.grid(True)
            plt.tight_layout()
            plt.show()

            rospy.loginfo("✅ STEP 4 Complete: Plots shown.")

        except Exception as e:
            rospy.logerr(f"🚫 STEP 4 Failed: {e}")

    def shutdown(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()


# ─────────────────────────────────────────────────────────────
# Main Execution Entry Point
# ─────────────────────────────────────────────────────────────
def main():
    rospy.init_node('friction_estimator_node')
    node = FrictionEstimatorNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()