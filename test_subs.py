#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox, RadioButtons
import numpy as np

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

# --- URDF joint limits (from your URDF) ---
URDF_LIMITS = {
    "L1": (-1.7, 1.7),
    "L2": (-0.98, 1.0),
    "L3": (-2.0, 1.3),
    "L4": (-2.0, 2.0),
    "L5": (-2.1, 2.1),
    "L6": (-3.1, 3.1),
}   


class JointPlotter(Node):
    def __init__(self):
        super().__init__('joint_plotter_with_sliders')

        # ROS interfaces
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # Joint configuration
        self.joint_names = list(URDF_LIMITS.keys())
        self.current_joint = self.joint_names[0]
        self.joint_commands = {name: 0.0 for name in self.joint_names}  # stored in radians
        self.joint_limits = URDF_LIMITS.copy()
        self.use_degrees = True  # initial mode: degrees
        self.suppress_publish = False  # prevent publishing during slider update

        # Plot buffers
        self.time_buffer, self.pos_buffer, self.vel_buffer, self.acc_buffer = [], [], [], []
        self.window = 10.0
        self.paused = False

        # --- Setup Matplotlib ---
        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
        plt.subplots_adjust(bottom=0.35, left=0.25, top=0.95)

        # Lines
        self.line_pos, = self.axs[0].plot([], [], label="Position")
        self.line_vel, = self.axs[1].plot([], [], label="Velocity")
        self.line_acc, = self.axs[2].plot([], [], label="Acceleration")

        for ax in self.axs:
            ax.legend()
            ax.grid(True)

        # --- GUI Widgets ---
        # Radio buttons for joint selection
        ax_joint_select = plt.axes([0.05, 0.35, 0.15, 0.5])
        self.radio = RadioButtons(ax_joint_select, self.joint_names, active=0)
        self.radio.on_clicked(self.joint_select)

        # Slider
        ax_slider = plt.axes([0.25, 0.25, 0.6, 0.03])
        lower, upper = self._get_slider_limits(self.current_joint)
        self.slider = Slider(ax_slider, "Joint Cmd (°)", lower, upper, valinit=0, valstep=1)
        self.slider.on_changed(self.publish_command)

        # Toggle Deg/Rad button
        ax_toggle = plt.axes([0.85, 0.25, 0.1, 0.04])
        self.toggle_button = Button(ax_toggle, 'Deg')
        self.toggle_button.on_clicked(self.toggle_unit)

        # Pause button
        ax_pause = plt.axes([0.85, 0.2, 0.1, 0.04])
        self.pause_button = Button(ax_pause, 'Pause')
        self.pause_button.on_clicked(self.toggle_pause)

        # Textbox for manual entry
        ax_textbox = plt.axes([0.25, 0.15, 0.4, 0.05])
        self.text_box = TextBox(ax_textbox, 'Enter Angle', initial="0")

        ax_send = plt.axes([0.7, 0.15, 0.15, 0.05])
        self.send_button = Button(ax_send, 'Send')
        self.send_button.on_clicked(self.send_from_text)

        # Text overlays moved to middle-left
        self.text_pos = self.axs[0].text(0.05, 0.5, '', transform=self.axs[0].transAxes, ha='left', fontsize=10)
        self.text_vel = self.axs[1].text(0.05, 0.5, '', transform=self.axs[1].transAxes, ha='left', fontsize=10)
        self.text_acc = self.axs[2].text(0.05, 0.5, '', transform=self.axs[2].transAxes, ha='left', fontsize=10)
        self.pause_text = self.axs[0].text(0.05, 0.85, '', transform=self.axs[0].transAxes, ha='left', color='red')

        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50)

    # --- Utility functions ---
    def _get_slider_limits(self, joint_name):
        lower, upper = self.joint_limits[joint_name]
        if self.use_degrees:
            lower *= RAD2DEG
            upper *= RAD2DEG
        return lower, upper

    # --- ROS callback ---
    def joint_callback(self, msg: JointState):
        if self.paused:
            return
        try:
            idx = msg.name.index(self.current_joint)
        except ValueError:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pos = msg.position[idx] * RAD2DEG if self.use_degrees else msg.position[idx]
        vel = msg.velocity[idx] * RAD2DEG if self.use_degrees else msg.velocity[idx]
        acc = msg.effort[idx] * RAD2DEG if self.use_degrees else msg.effort[idx]
        self.time_buffer.append(t)
        self.pos_buffer.append(pos)
        self.vel_buffer.append(vel)
        self.acc_buffer.append(acc)
        t_min = t - self.window
        while self.time_buffer and self.time_buffer[0] < t_min:
            self.time_buffer.pop(0)
            self.pos_buffer.pop(0)
            self.vel_buffer.pop(0)
            self.acc_buffer.pop(0)

    # --- Publish commands ---
    def publish_command(self, val):
        if self.suppress_publish:
            return
        lower, upper = self._get_slider_limits(self.current_joint)
        val_clamped = max(lower, min(val, upper))
        self.joint_commands[self.current_joint] = val_clamped * DEG2RAD if self.use_degrees else val_clamped
        msg = Float64MultiArray()
        msg.data = [self.joint_commands[j] for j in self.joint_names]
        self.joint_pub.publish(msg)
        self.text_box.set_val(f"{val_clamped:.2f}")

    # --- GUI callbacks ---
    def toggle_unit(self, event):
        self.use_degrees = not self.use_degrees
        self.toggle_button.label.set_text('Deg' if self.use_degrees else 'Rad')
        val = self.joint_commands[self.current_joint] * RAD2DEG if self.use_degrees else self.joint_commands[self.current_joint]
        lower, upper = self._get_slider_limits(self.current_joint)
        self.suppress_publish = True
        self.slider.valmin = lower
        self.slider.valmax = upper
        self.slider.ax.set_xlim(lower, upper)
        self.slider.set_val(val)
        self.slider.ax.figure.canvas.draw_idle()
        self.text_box.set_val(f"{val:.2f}")
        self.suppress_publish = False

    def toggle_pause(self, event):
        self.paused = not self.paused
        self.pause_button.label.set_text('Resume' if self.paused else 'Pause')

    def send_from_text(self, event):
        try:
            val = float(self.text_box.text)
            lower, upper = self._get_slider_limits(self.current_joint)
            if lower <= val <= upper:
                self.slider.set_val(val)
                self.publish_command(val)
            else:
                self.get_logger().warn(f"Value {val} out of limits")
        except ValueError:
            self.get_logger().warn(f"Invalid input: {self.text_box.text}")

    def joint_select(self, label):
        self.current_joint = label
        self.time_buffer.clear()
        self.pos_buffer.clear()
        self.vel_buffer.clear()
        self.acc_buffer.clear()
        lower, upper = self._get_slider_limits(label)
        val = self.joint_commands[label] * RAD2DEG if self.use_degrees else self.joint_commands[label]
        self.suppress_publish = True
        self.slider.valmin = lower
        self.slider.valmax = upper
        self.slider.ax.set_xlim(lower, upper)
        self.slider.set_val(val)
        self.slider.ax.figure.canvas.draw_idle()
        self.text_box.set_val(f"{val:.2f}")
        self.suppress_publish = False

    # --- Animation ---
    def update_plot(self, frame):
        if len(self.time_buffer) < 2:
            return self.line_pos, self.line_vel, self.line_acc, self.text_pos, self.text_vel, self.text_acc, self.pause_text
        t = np.array(self.time_buffer) - self.time_buffer[-1]
        self.line_pos.set_data(t, self.pos_buffer)
        self.line_vel.set_data(t, self.vel_buffer)
        self.line_acc.set_data(t, self.acc_buffer)
        unit = "°" if self.use_degrees else "rad"
        self.text_pos.set_text(f"{self.pos_buffer[-1]:.2f}{unit}")
        self.text_vel.set_text(f"{self.vel_buffer[-1]:.2f}{unit}/s")
        self.text_acc.set_text(f"{self.acc_buffer[-1]:.2f}{unit}/s²")
        self.pause_text.set_text('PAUSED' if self.paused else '')
        for ax in self.axs:
            ax.set_xlim(-self.window, 0)
            ax.relim()
            ax.autoscale_view(scaley=True)
        return self.line_pos, self.line_vel, self.line_acc, self.text_pos, self.text_vel, self.text_acc, self.pause_text


def main():
    rclpy.init()
    node = JointPlotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
