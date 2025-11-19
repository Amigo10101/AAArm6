import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk
from scipy.linalg import solve_continuous_are

# --- Simulation parameters ---
dt = 0.001
target_position = 0.0
mass = 1.0
damping = 0.1

# --- State-space ---
A = np.array([[0, 1],
              [0, -damping/mass]])
B = np.array([[0],
              [1/mass]])
C = np.array([[1, 0]])  # position measurement

# LQR design
Q = np.array([[100, 0],
              [0, 1]])
R = np.array([[1.0]])
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

# Kalman filter design
Q_kf = np.array([[1e-5, 0],
                 [0, 1e-3]])
R_kf = np.array([[1e-2]])
x_hat = np.array([[0.0],
                  [0.0]])
P_kf = np.eye(2)

# --- Simulation state ---
position = 1.0
velocity = 0.0
external_position_shift = 0.0

# --- Tkinter window ---
root = tk.Tk()
root.title("LQR with Kalman Filter")

# --- Matplotlib figure ---
fig, axs = plt.subplots(3, 1, figsize=(6,6))
plt.tight_layout(pad=3)
lines = []
labels = ["Position", "Velocity (est)", "Acceleration (est)"]
colors = ["blue", "orange", "green"]
for ax, label, color in zip(axs, labels, colors):
    line, = ax.plot([], [], color=color, lw=2)
    ax.set_ylabel(label)
    ax.grid(True)
    lines.append(line)
axs[-1].set_xlabel("Time (s)")

# --- Embed Matplotlib in Tkinter ---
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# --- Slider ---
def on_slider(val):
    global external_position_shift
    external_position_shift = float(val)

ttk.Label(root, text="External Position Shift").pack()
slider = ttk.Scale(root, from_=-2.0, to=2.0, orient="horizontal", command=on_slider, length=400)
slider.pack()
slider.set(0.0)

# --- Numeric labels ---
value_frame = tk.Frame(root)
value_frame.pack(pady=5)
pos_label = ttk.Label(value_frame, text=f"Position: {float(position):.4f}")
vel_label = ttk.Label(value_frame, text=f"Velocity (est): 0.0")
acc_label = ttk.Label(value_frame, text=f"Acceleration (est): 0.0")
pos_label.grid(row=0, column=0, padx=10)
vel_label.grid(row=0, column=1, padx=10)
acc_label.grid(row=0, column=2, padx=10)

# --- History ---
time_hist, pos_hist, vel_hist, acc_hist = [], [], [], []

# --- Simulation update ---
frame_count = 0
def update(frame):
    global position, velocity, x_hat, P_kf, frame_count

    frame_count += 1
    t = frame_count * dt

    # Measurement
    z = float(position + external_position_shift)

    # --- Kalman Filter Prediction ---
    x_hat = (A @ x_hat * dt + x_hat)  # no control applied in prediction
    P_kf = A @ P_kf @ A.T + Q_kf

    # --- Kalman Filter Update ---
    y = z - float((C @ x_hat).item())
    S = float(C @ P_kf @ C.T + R_kf)
    K_kf = (P_kf @ C.T / S)
    x_hat = x_hat + K_kf * y
    P_kf = (np.eye(2) - K_kf @ C) @ P_kf

    # --- LQR Control ---
    u = -float((K @ x_hat).item())
    max_force = 1.0
    u = np.clip(u, -max_force, max_force)

    # --- Apply control and simple physics ---
    acc = (u - damping*velocity)/mass
    velocity += acc * dt
    position += velocity * dt

    # --- Record history ---
    time_hist.append(t)
    pos_hist.append(position)
    vel_hist.append(float(x_hat[1,0]))
    acc_hist.append(acc)

    if len(time_hist) > 1000:
        time_hist.pop(0)
        pos_hist.pop(0)
        vel_hist.pop(0)
        acc_hist.pop(0)

    # --- Update plots ---
    data_lists = [pos_hist, vel_hist, acc_hist]
    for line, data, ax in zip(lines, data_lists, axs):
        line.set_data(time_hist, data)
        ax.relim()
        ax.autoscale_view()

    # --- Update numeric labels ---
    pos_label.config(text=f"Position: {float(position):.4f}")
    vel_label.config(text=f"Velocity (est): {float(x_hat[1,0]):.4f}")
    acc_label.config(text=f"Acceleration (est): {float(acc):.4f}")

    canvas.draw()
    return lines

# --- Animation ---
ani = FuncAnimation(fig, update, interval=dt*1000, blit=False)
root.mainloop()
