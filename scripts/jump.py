#!/usr/bin/env python3

import math
import rospy

from std_srvs.srv import Trigger, TriggerResponse
from climbingrobot_hardware_interface.srv import AlpineBodyCommand, AlpineBodyCommandResponse
from std_msgs.msg import Float32, String
from climbingrobot_hardware_interface.msg import RopeCommand


class JumpNode:

    def __init__(self):
        # ── Publishers ───────────────────────────────────────────────────
        self.pub_s1 = rospy.Publisher('/alpine/dongle/servoValve1', Float32, queue_size=10)
        self.pub_s2 = rospy.Publisher('/alpine/dongle/servoValve2', Float32, queue_size=10)
        self.pub_left = rospy.Publisher('/winch/left/command', RopeCommand, queue_size=10)
        self.pub_right = rospy.Publisher('/winch/right/command', RopeCommand, queue_size=10)
        self.pub_left_mode = rospy.Publisher('/winch/left/set_motor_mode', String, queue_size=10)
        self.pub_right_mode = rospy.Publisher('/winch/right/set_motor_mode', String, queue_size=10)

        # ── Services ─────────────────────────────────────────────────────
        rospy.Service('/alpine/jump', Trigger, self.handle_jump)
        rospy.Service('/alpine/jump_abort', Trigger, self.handle_abort)
        rospy.Service('/alpine/jump_stop', Trigger, self.handle_stop)

        # Compatibility with climbingrobot_controller2_real.py.
        # The real controller calls /alpine_body/command with leg_force/contact_normal.
        rospy.Service('/alpine_body/command', AlpineBodyCommand, self.handle_alpine_body_command)

        # ── Timer: 100 Hz state machine ──────────────────────────────────
        self.timer = rospy.Timer(rospy.Duration(0.01), self.tick)

        # ── Force / velocity settings ────────────────────────────────────
        self.up_force = 25.0
        self.hold_force = 15.0
        self.rewind_velocity = 0.0
        self.settle_velocity = 0.0

        # ── State ────────────────────────────────────────────────────────
        self.sequence_running = False
        self.sequence_start_ms = 0.0
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None
        self.current_mode = None

        # If True, jump.py publishes winch commands during the sequence.
        # This is used for manual /alpine/jump.
        #
        # If False, jump.py only publishes valve commands.
        # This is used when climbingrobot_controller2_real.py calls
        # /alpine_body/command and the optimized Fr_l / Fr_r should come
        # from the high-level controller, not from this local hardcoded sequence.
        self.command_ropes_during_sequence = True

        # Store last leg force received from climbingrobot_controller2_real.py.
        # At the moment this node does not convert Fleg into valve command;
        # the valve sequence is still timing-based.
        self.last_body_leg_force = float('nan')

        # ── Sequence: (duration_ms, mode, lf, rf, lv, rv, s1, s2) ───────
        #
        # IMPORTANT:
        # lf/rf are only used when command_ropes_during_sequence=True.
        # When /alpine_body/command triggers the jump, this node sends only
        # servo valve commands and does not overwrite optimized rope forces.
        NAN = float('nan')
        self.sequence = [
            # Phase 1: piston only, ropes idle
            (220, 'torque', 0.0, 0.0, NAN, NAN, 90.0, 0.0),

            # Phase 2: ropes pull hard while airborne
            (650, 'torque', -self.up_force, -self.up_force, NAN, NAN, 0.0, 90.0),

            # Phase 3: pull slightly less, valve2 still open
            (400, 'torque', -18.0, -18.0, NAN, NAN, 0.0, 90.0),

            # Phase 4: final hold
            (1500, 'torque', -self.hold_force, -self.hold_force, NAN, NAN, 0.0, 0.0),
        ]

        # Precompute cumulative timeline
        self.timeline = []
        acc = 0.0
        for dur, mode, lf, rf, lv, rv, s1, s2 in self.sequence:
            acc += float(dur)
            self.timeline.append((acc, mode, lf, rf, lv, rv, s1, s2))

        rospy.logwarn(
            "jump_node SAFE started: no winch/valve command is sent until a jump service is called"
        )
        rospy.loginfo(
            f"settings: up_force={self.up_force}, hold_force={self.hold_force}, "
            f"rewind_velocity={self.rewind_velocity}, settle_velocity={self.settle_velocity}"
        )

    # ────────────────────────────────────────────────────────────────────
    # Time helper
    # ────────────────────────────────────────────────────────────────────

    def now_ms(self) -> float:
        return rospy.Time.now().to_sec() * 1000.0

    # ────────────────────────────────────────────────────────────────────
    # Mode helpers
    # ────────────────────────────────────────────────────────────────────

    @staticmethod
    def safe_round(x, ndigits: int = 3):
        if isinstance(x, float) and math.isnan(x):
            return 'nan'
        return round(float(x), ndigits)

    def set_torque_mode(self):
        msg = String(data='closed_loop_torque')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'torque'
        rospy.loginfo("Published closed_loop_torque to both winches")

    def set_velocity_mode(self):
        msg = String(data='closed_loop_velocity')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'velocity'
        rospy.loginfo("Published closed_loop_velocity to both winches")

    def set_idle_mode(self):
        msg = String(data='idle')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'idle'
        rospy.logwarn("Published idle to both winches")

    def set_mode(self, mode: str):
        if mode == self.current_mode:
            return

        if mode == 'torque':
            self.set_torque_mode()
        elif mode == 'velocity':
            self.set_velocity_mode()
        elif mode == 'idle':
            self.set_idle_mode()
        else:
            rospy.logwarn(f"Unknown mode requested: {mode}")

    # ────────────────────────────────────────────────────────────────────
    # Sequence start helper
    # ────────────────────────────────────────────────────────────────────

    def start_jump_sequence(self, command_ropes=True):
        """
        Start the timed jump sequence.

        command_ropes=True:
            Used by manual /alpine/jump.
            jump.py commands both valves and winches using the local safe sequence.

        command_ropes=False:
            Used by /alpine_body/command from climbingrobot_controller2_real.py.
            jump.py commands only the valves.
            Rope commands are expected from the high-level controller, where
            optimized Fr_l / Fr_r are computed.
        """
        self.sequence_running = True
        self.sequence_start_ms = self.now_ms()
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None
        self.command_ropes_during_sequence = bool(command_ropes)

        if self.command_ropes_during_sequence:
            self.set_torque_mode()
        else:
            rospy.logwarn(
                "Starting jump sequence in VALVES-ONLY mode: "
                "winch commands will not be published by jump.py"
            )

        rospy.loginfo(
            "Jump sequence triggered: command_ropes=%s",
            str(self.command_ropes_during_sequence)
        )

    # ────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ────────────────────────────────────────────────────────────────────

    def publish_all(self, lf, rf, lv, rv, s1, s2):
        # Always publish valve commands.
        self.pub_s1.publish(Float32(data=float(s1)))
        self.pub_s2.publish(Float32(data=float(s2)))

        # If the high-level real controller triggered this sequence through
        # /alpine_body/command, do not overwrite optimized Fr_l / Fr_r.
        if not self.command_ropes_during_sequence:
            return

        now = rospy.Time.now()

        left = RopeCommand()
        left.header.stamp = now
        left.rope_force = float(lf)
        left.rope_velocity = float(lv)
        left.rope_position = float('nan')

        right = RopeCommand()
        right.header.stamp = now
        right.rope_force = float(rf)
        right.rope_velocity = float(rv)
        right.rope_position = float('nan')

        self.pub_left.publish(left)
        self.pub_right.publish(right)

    def send_if_changed(self, mode, lf, rf, lv, rv, s1, s2):
        cmd = (
            mode,
            self.safe_round(lf),
            self.safe_round(rf),
            self.safe_round(lv),
            self.safe_round(rv),
            self.safe_round(s1),
            self.safe_round(s2),
            self.command_ropes_during_sequence,
        )

        if cmd != self.last_sent:
            if self.command_ropes_during_sequence:
                self.set_mode(mode)

            self.publish_all(lf, rf, lv, rv, s1, s2)
            self.last_sent = cmd

    def refresh_current(self, lf, rf, lv, rv, s1, s2):
        self.publish_all(lf, rf, lv, rv, s1, s2)

    def publish_hold_light(self):
        NAN = float('nan')
        self.command_ropes_during_sequence = True
        self.send_if_changed('torque', -self.hold_force, -self.hold_force, NAN, NAN, 0.0, 0.0)

    def publish_zero_force(self):
        NAN = float('nan')
        self.command_ropes_during_sequence = True
        self.send_if_changed('torque', 0.0, 0.0, NAN, NAN, 0.0, 0.0)

    def publish_valves_zero(self):
        self.pub_s1.publish(Float32(data=0.0))
        self.pub_s2.publish(Float32(data=0.0))

    # ────────────────────────────────────────────────────────────────────
    # 100 Hz tick
    # ────────────────────────────────────────────────────────────────────

    def tick(self, event):
        # SAFETY: do nothing until a jump service is called
        if not self.sequence_running:
            return

        elapsed_ms = self.now_ms() - self.sequence_start_ms

        NAN = float('nan')
        phase_index = None
        mode = 'torque'
        lf, rf = -self.hold_force, -self.hold_force
        lv, rv = NAN, NAN
        s1, s2 = 0.0, 0.0

        for i, (limit_ms, mode_v, lf_v, rf_v, lv_v, rv_v, s1_v, s2_v) in enumerate(self.timeline):
            if elapsed_ms < limit_ms:
                phase_index = i
                mode = mode_v
                lf, rf = lf_v, rf_v
                lv, rv = lv_v, rv_v
                s1, s2 = s1_v, s2_v
                break

        if phase_index is None:
            self.sequence_running = False
            self.current_phase_index = -1
            self.phase_refresh_div = 0
            self.last_sent = None

            if self.command_ropes_during_sequence:
                rospy.loginfo("Jump sequence completed -> light hold")
                self.publish_hold_light()
            else:
                rospy.loginfo("Valves-only jump sequence completed -> valves closed, no winch command sent")
                self.publish_valves_zero()

            return

        if phase_index != self.current_phase_index:
            self.current_phase_index = phase_index
            self.phase_refresh_div = 0

            rospy.loginfo(
                f"Phase {phase_index + 1}/{len(self.timeline)} -> "
                f"mode={mode}, lf={lf}, rf={rf}, lv={lv}, rv={rv}, "
                f"s1={s1:.1f}, s2={s2:.1f}, "
                f"command_ropes={self.command_ropes_during_sequence}"
            )

            self.send_if_changed(mode, lf, rf, lv, rv, s1, s2)
            return

        self.phase_refresh_div += 1
        if self.phase_refresh_div >= 3:
            self.phase_refresh_div = 0
            self.refresh_current(lf, rf, lv, rv, s1, s2)

    # ────────────────────────────────────────────────────────────────────
    # Service handlers
    # ────────────────────────────────────────────────────────────────────

    def handle_alpine_body_command(self, req):
        """
        Called by climbingrobot_controller2_real.py.

        The high-level controller computes Fleg and optimized rope forces
        Fr_l / Fr_r. This service is used here only to trigger the pneumatic
        valve timing. The winch commands must not be overwritten by this node.
        """
        self.last_body_leg_force = float(req.leg_force)

        rospy.logwarn(
            "[alpine_body/command] received from high-level controller: "
            "leg_force=%.3f, contact_normal=(%.3f, %.3f, %.3f). "
            "Triggering VALVES-ONLY jump sequence; optimized rope forces are expected from high-level controller.",
            float(req.leg_force),
            float(req.contact_normal.x),
            float(req.contact_normal.y),
            float(req.contact_normal.z),
        )

        try:
            self.start_jump_sequence(command_ropes=False)
            return AlpineBodyCommandResponse(ack=True)
        except Exception as e:
            rospy.logerr(f"[alpine_body/command] failed: {e}")
            return AlpineBodyCommandResponse(ack=False)

    def handle_jump(self, req):
        """
        Manual safe jump.

        This still commands both valves and winches using the local hardcoded
        safe sequence.
        """
        self.start_jump_sequence(command_ropes=True)
        return TriggerResponse(success=True, message="Jump sequence started with local rope commands")

    def handle_abort(self, req):
        self.sequence_running = False
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.set_torque_mode()
        self.publish_hold_light()

        rospy.logwarn("Jump aborted -> light hold torque sent")
        return TriggerResponse(success=True, message="Jump aborted, light hold sent")

    def handle_stop(self, req):
        self.sequence_running = False
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.set_torque_mode()
        self.publish_zero_force()
        self.publish_valves_zero()

        rospy.logwarn("Jump stopped -> zero force sent and valves closed")
        return TriggerResponse(success=True, message="Jump stopped, zero force sent and valves closed")


def main():
    rospy.init_node('jump_node')
    node = JumpNode()
    rospy.spin()


if __name__ == '__main__':
    main()
