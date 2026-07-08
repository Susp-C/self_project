import rospy
import numpy as np
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy


class InputInterface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0

        self.rounding_dp = 2

        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0

        # --- Posture smoothing factor (0..1) ---
        # Higher  = snappier response and faster return-to-center.
        # Lower   = slower, smoother motion.
        # This is NOT an integrator: the target is always the current stick
        # value, so releasing the stick smoothly returns posture to zero.
        self.posture_smooth = 0.15

        # Max posture angles (radians). Fall back to sensible defaults if the
        # config does not define them.
        self.max_roll = getattr(self.config, "max_roll", 0.3)
        self.max_pitch = getattr(self.config, "max_pitch", 0.35)

        # Smoothed posture outputs (start level).
        self.smoothed_roll = 0.0
        self.smoothed_pitch = 0.0

        self.input_messages = rospy.Subscriber("joy", Joy, self.input_callback)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

    def input_callback(self, msg):
        self.developing_command = Command()

        ####### Handle discrete commands ########
        # Trot toggle (R1)
        gait_toggle = msg.buttons[5]
        if self.trot_event != 1:
            self.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        # Hop toggle (X)
        hop_toggle = msg.buttons[0]
        if self.hop_event != 1:
            self.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)

        # Joystick control toggle (L1)
        joystick_toggle = msg.buttons[4]
        if self.joystick_control_event != 1:
            self.joystick_control_event = (joystick_toggle == 1 and self.previous_joystick_toggle == 0)

        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_joystick_toggle = joystick_toggle

        ####### Handle continuous commands ########
        # Translation (left stick)
        x_vel = msg.axes[1] * self.config.max_x_velocity   # LY -> forward/back
        y_vel = msg.axes[0] * self.config.max_y_velocity   # LX -> strafe
        self.developing_command.horizontal_velocity = np.round(
            np.array([x_vel, y_vel]), self.rounding_dp)

        # Turning (right stick horizontal) -> yaw rate
        self.developing_command.yaw_rate = np.round(
            msg.axes[3], self.rounding_dp) * self.config.max_yaw_rate  # RX

        # Height (D-pad vertical) -> keep as incremental (four legs move
        # together, so it never causes tilt/imbalance).
        self.developing_command.height_movement = np.round(msg.axes[5], self.rounding_dp)  # DPAD Y

        # --- Posture targets: DIRECT position targets, NO integration ---
        # Right stick vertical (RY, axes[4]) -> pitch/roll targets.
        # These are absolute targets: releasing the stick sets target to 0.
        self.developing_command.pitch_target = np.round(msg.axes[4], self.rounding_dp) * self.max_pitch
        self.developing_command.roll_target = -np.round(msg.axes[4], self.rounding_dp) * self.max_roll

        self.new_command = self.developing_command

    def get_command(self, state, message_rate):

        self.current_command = self.new_command

        self.current_command.trot_event = self.trot_event
        self.current_command.hop_event = self.hop_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0

        message_dt = 1.0 / message_rate

        # --- Roll / Pitch: smooth follow toward stick target (NO integrator) ---
        # Read desired targets (default to 0 if not set this cycle).
        roll_target = getattr(self.current_command, "roll_target", 0.0)
        pitch_target = getattr(self.current_command, "pitch_target", 0.0)

        # Deadband so tiny stick noise resolves cleanly to level.
        roll_target = deadband(roll_target, self.config.pitch_deadband)
        pitch_target = deadband(pitch_target, self.config.pitch_deadband)

        # Exponential smoothing toward the target.
        # target = stick value; when stick released target = 0 -> returns to
        # center smoothly. This is a low-pass filter, not an accumulator.
        self.smoothed_roll += self.posture_smooth * (roll_target - self.smoothed_roll)
        self.smoothed_pitch += self.posture_smooth * (pitch_target - self.smoothed_pitch)

        # Snap very small residuals to exactly zero so posture is truly level.
        if abs(self.smoothed_roll) < 1e-3:
            self.smoothed_roll = 0.0
        if abs(self.smoothed_pitch) < 1e-3:
            self.smoothed_pitch = 0.0

        self.current_command.roll = float(np.clip(self.smoothed_roll, -0.3, 0.3))
        self.current_command.pitch = float(np.clip(self.smoothed_pitch, -0.35, 0.35))

        # --- Height: keep incremental (safe, no tilt) ---
        self.current_command.height = np.clip(
            state.height - message_dt * self.config.z_speed * self.current_command.height_movement,
            -0.27, -0.08)

        return self.current_command
