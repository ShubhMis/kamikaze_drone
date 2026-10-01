import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -----------------------------
# Simulation Parameters
# -----------------------------
# N = Navigation Constant
# Controls aggressiveness of turning.
# Typical PN values: 3–5
N = 5.0

# dt = simulation timestep
dt = 0.05

# Constant speeds (pure turning model)
missile_speed = 4.0
target_speed = 3.0


# -----------------------------
# Initial States
# -----------------------------
# Missile initial position and velocity
missile_pos = np.array([100.0, 300.0])
missile_vel = np.array([missile_speed, 0.0])

# Target initial position and velocity
target_pos = np.array([0.0, 0.0])
target_vel = np.array([0.0, 0.0])

# Waypoint system for target motion
waypoints = []
current_wp = 0
running = False

# Trajectory history for plotting
missile_traj = []
target_traj = []

waypoint_scatter = None  # Safe scatter reference


# -----------------------------
# Setup Plot
# -----------------------------
fig, ax = plt.subplots()
ax.set_xlim(0, 800)
ax.set_ylim(0, 600)
ax.set_title("PN Missile Simulator\nClick: Waypoints | Enter: Start | R: Reset")

missile_dot, = ax.plot([], [], 'ro', label="Missile")
target_dot, = ax.plot([], [], 'bo', label="Target")
trajectory_line, = ax.plot([], [], 'r-')
target_path_line, = ax.plot([], [], 'b--')

ax.legend()


# -----------------------------
# Initialize Target
# -----------------------------
def initialize_target():
    global target_pos, target_vel, current_wp, target_traj

    current_wp = 0

    # Start at first waypoint
    target_pos[:] = waypoints[0]

    # Start with zero velocity (will update next frame)
    target_vel[:] = 0

    target_traj = []


# -----------------------------
# Mouse Click → Add Waypoints
# -----------------------------
def onclick(event):
    if event.inaxes:
        waypoints.append(np.array([event.xdata, event.ydata]))
        update_waypoint_plot()


def update_waypoint_plot():
    global waypoint_scatter

    # Remove previous waypoint markers safely
    if waypoint_scatter is not None:
        waypoint_scatter.remove()
        waypoint_scatter = None

    if waypoints:
        wp = np.array(waypoints)

        # Draw dashed path between waypoints
        target_path_line.set_data(wp[:, 0], wp[:, 1])

        # Draw waypoint markers
        waypoint_scatter = ax.scatter(wp[:, 0], wp[:, 1], c='blue', s=40)

    fig.canvas.draw_idle()


# -----------------------------
# Key Press
# -----------------------------
def onkey(event):
    global running

    if event.key == 'enter':
        if len(waypoints) > 1:
            initialize_target()
            running = True
        else:
            print("Add at least TWO waypoints before starting.")

    elif event.key == 'r':
        reset()


# -----------------------------
# Reset Simulation
# -----------------------------
def reset():
    global running, current_wp, waypoint_scatter

    # Reset missile
    missile_pos[:] = [100.0, 300.0]
    missile_vel[:] = [missile_speed, 0.0]

    # Reset target
    target_pos[:] = 0
    target_vel[:] = 0

    # Clear state
    waypoints.clear()
    missile_traj.clear()
    target_traj.clear()

    current_wp = 0
    running = False

    # Clear plot visuals
    missile_dot.set_data([], [])
    target_dot.set_data([], [])
    trajectory_line.set_data([], [])
    target_path_line.set_data([], [])

    if waypoint_scatter is not None:
        waypoint_scatter.remove()
        waypoint_scatter = None

    fig.canvas.draw_idle()


# -----------------------------
# Target Waypoint Motion
# -----------------------------
def update_target():
    global current_wp

    # If final waypoint reached → stop target
    if current_wp >= len(waypoints) - 1:
        target_vel[:] = 0
        return

    next_wp = waypoints[current_wp + 1]

    # Direction from target to next waypoint
    direction = next_wp - target_pos
    dist = np.linalg.norm(direction)

    # If close enough → switch to next waypoint
    if dist < 5:
        current_wp += 1
        return

    # Normalize direction vector
    direction /= dist

    # Move target at constant speed
    target_vel[:] = direction * target_speed
    target_pos[:] += target_vel * dt


# -----------------------------
# Proportional Navigation
# -----------------------------
def update_missile():

    # ---------------------------------------------------------
    # 1) Relative Geometry
    # ---------------------------------------------------------
    # Vector from missile to target
    rel_pos = target_pos - missile_pos

    # Relative velocity between target and missile
    rel_vel = target_vel - missile_vel

    # Distance between them
    dist = np.linalg.norm(rel_pos)

    # Intercept condition
    if dist < 2:
        print("Target Intercepted!")
        return False


    # ---------------------------------------------------------
    # 2) Compute Line-Of-Sight (LOS) Rate
    # ---------------------------------------------------------
    # PN does NOT aim directly at target.
    #
    # Instead, it measures how fast the line-of-sight
    # angle is rotating.
    #
    # λ_dot = (r × v_rel) / |r|²
    #
    # In 2D, cross product becomes scalar:
    # r_x * v_y − r_y * v_x
    los_rate = (rel_pos[0]*rel_vel[1] -
                rel_pos[1]*rel_vel[0]) / (dist**2)


    # ---------------------------------------------------------
    # 3) Proportional Navigation Law
    # ---------------------------------------------------------
    # Core PN equation:
    #
    # a_command = N * V_missile * λ_dot
    #
    # N = navigation constant
    # V = missile speed
    #
    # This produces lateral acceleration
    # proportional to LOS rotation rate.
    accel_mag = N * missile_speed * los_rate


    # ---------------------------------------------------------
    # 4) Apply Lateral Acceleration
    # ---------------------------------------------------------
    # Acceleration must be perpendicular
    # to missile velocity (pure turning).
    #
    # Perpendicular vector in 2D:
    # [-Vy, Vx]
    normal = np.array([-missile_vel[1], missile_vel[0]])

    norm_val = np.linalg.norm(normal)

    if norm_val != 0:
        normal /= norm_val


    # ---------------------------------------------------------
    # 5) Update Missile Velocity
    # ---------------------------------------------------------
    # v_new = v_old + a * dt
    missile_vel[:] += accel_mag * normal * dt

    # Keep speed constant (ideal missile assumption)
    missile_vel[:] = missile_speed * missile_vel / np.linalg.norm(missile_vel)


    # ---------------------------------------------------------
    # 6) Update Position
    # ---------------------------------------------------------
    # x_new = x_old + v * dt
    missile_pos[:] += missile_vel * dt

    return True


# -----------------------------
# Animation Update
# -----------------------------
def update(frame):
    global running

    if running:

        # Update target motion first
        update_target()

        # Update missile using PN guidance
        if not update_missile():
            running = False

        missile_traj.append(missile_pos.copy())

        if len(missile_traj) > 1:
            mt = np.array(missile_traj)
            trajectory_line.set_data(mt[:, 0], mt[:, 1])

        missile_dot.set_data([missile_pos[0]], [missile_pos[1]])
        target_dot.set_data([target_pos[0]], [target_pos[1]])

    return missile_dot, target_dot, trajectory_line


# -----------------------------
# Bind Events
# -----------------------------
fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', onkey)

ani = FuncAnimation(fig, update, interval=30, cache_frame_data=False)

plt.show()
