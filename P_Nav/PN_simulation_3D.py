import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D

# ==========================================================
#                 SIMULATION PARAMETERS
# ==========================================================

N = 3.0          # Navigation constant (aggressiveness of turn)
dt = 0.05        # Time step for numerical integration
missile_speed = 11.0
target_speed = 10.0

# ==========================================================
#                 INITIAL STATES
# ==========================================================

# Missile initial position (x,y,z)
missile_pos = np.array([100.0, 100.0, 200.0])

# Missile initial velocity (pointing along +X direction)
missile_vel = np.array([missile_speed, 0.0, 0.0])

# Target state (initialized later)
target_pos = np.zeros(3)
target_vel = np.zeros(3)

# Waypoint list for target motion
waypoints = []
current_wp = 0

# Simulation control
running = False

# Store missile trajectory for plotting
missile_traj = []

# ==========================================================
#                 FIGURE SETUP
# ==========================================================

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.25)

# Equal aspect ratio (VERY important for 3D realism)
ax.set_box_aspect([1,1,1])

# Camera view angle
ax.view_init(elev=25, azim=45)

# Axis limits
ax.set_xlim(0, 800)
ax.set_ylim(0, 600)
ax.set_zlim(0, 600)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Plot elements
missile_dot, = ax.plot([], [], [], 'ro', label="Missile")
target_dot,  = ax.plot([], [], [], 'bo', label="Target")
traj_line,   = ax.plot([], [], [], 'r-')
path_line,   = ax.plot([], [], [], 'b--')

ax.legend()

# ==========================================================
#           SLIDERS FOR TRUE 3D WAYPOINT INPUT
# ==========================================================

# Sliders allow true 3D waypoint selection
ax_x = plt.axes([0.2, 0.15, 0.6, 0.03])
ax_y = plt.axes([0.2, 0.1, 0.6, 0.03])
ax_z = plt.axes([0.2, 0.05, 0.6, 0.03])

slider_x = Slider(ax_x, "X", 0, 800, valinit=400)
slider_y = Slider(ax_y, "Y", 0, 600, valinit=300)
slider_z = Slider(ax_z, "Z", 0, 600, valinit=200)

# ==========================================================
#           ADD WAYPOINT BUTTON
# ==========================================================

ax_button = plt.axes([0.85, 0.05, 0.1, 0.08])
btn = Button(ax_button, "Add WP")

def add_waypoint(event):
    """
    Adds a new 3D waypoint for the target.
    Waypoint is taken directly from slider values.
    """
    wp = np.array([slider_x.val, slider_y.val, slider_z.val])
    waypoints.append(wp)
    update_path()

btn.on_clicked(add_waypoint)

# ==========================================================
#           UPDATE DISPLAYED TARGET PATH
# ==========================================================

def update_path():
    """
    Draw dashed line showing target waypoint path.
    """
    if waypoints:
        wp = np.array(waypoints)
        path_line.set_data(wp[:,0], wp[:,1])
        path_line.set_3d_properties(wp[:,2])
        fig.canvas.draw_idle()

# ==========================================================
#           INITIALIZE TARGET MOTION
# ==========================================================

def initialize_target():
    """
    Sets target position to first waypoint
    and resets waypoint index.
    """
    global target_pos, target_vel, current_wp
    current_wp = 0
    target_pos[:] = waypoints[0]
    target_vel[:] = 0

# ==========================================================
#           TARGET MOTION BETWEEN WAYPOINTS
# ==========================================================

def update_target():
    """
    Moves target in straight lines between waypoints.
    """
    global current_wp

    # Stop if last waypoint reached
    if current_wp >= len(waypoints)-1:
        target_vel[:] = 0
        return

    next_wp = waypoints[current_wp+1]

    # Direction vector to next waypoint
    direction = next_wp - target_pos
    dist = np.linalg.norm(direction)

    # Switch to next waypoint if close enough
    if dist < 0.1:
        current_wp += 1
        return

    # Normalize direction
    direction /= dist

    # Move at constant speed
    target_vel[:] = direction * target_speed
    target_pos[:] += target_vel * dt

# ==========================================================
#           3D PROPORTIONAL NAVIGATION GUIDANCE
# ==========================================================

def update_missile():
    """
    Implements full 3D Proportional Navigation.

    Steps:
    1) Compute relative position
    2) Compute relative velocity
    3) Compute LOS angular rate
    4) Compute commanded acceleration
    5) Update missile velocity & position
    """

    # Relative position vector (target wrt missile)
    rel_pos = target_pos - missile_pos

    # Relative velocity
    rel_vel = target_vel - missile_vel

    dist = np.linalg.norm(rel_pos)

    # Intercept condition
    if dist < 1:
        print("Intercept!")
        return False

    # Line-of-sight angular rate vector
    omega   = np.cross(rel_pos, rel_vel) / (dist**2)

    # Missile velocity unit vector
    v_hat = missile_vel / np.linalg.norm(missile_vel)

    # Proportional Navigation acceleration command
    accel = N * missile_speed * np.cross(omega, v_hat)
    print(accel)

    # Integrate acceleration
    missile_vel[:] += accel * dt

    # Keep missile speed constant
    missile_vel[:] = missile_speed * missile_vel / np.linalg.norm(missile_vel)

    # Update position
    missile_pos[:] += missile_vel * dt

    return True

# ==========================================================
#           KEYBOARD CONTROLS
# ==========================================================

def onkey(event):
    global running

    if event.key == 'enter':
        if len(waypoints) > 1:
            initialize_target()
            running = True

    if event.key == 'r':
        reset()

fig.canvas.mpl_connect('key_press_event', onkey)

# ==========================================================
#           RESET SIMULATION
# ==========================================================

def reset():
    global running, waypoints, missile_traj

    missile_pos[:] = [100,300,200]
    missile_vel[:] = [missile_speed,0,0]
    target_pos[:] = 0
    target_vel[:] = 0

    waypoints.clear()
    missile_traj.clear()
    running = False

    traj_line.set_data([],[])
    traj_line.set_3d_properties([])
    path_line.set_data([],[])
    path_line.set_3d_properties([])

    fig.canvas.draw_idle()

# ==========================================================
#           ANIMATION LOOP
# ==========================================================

def update(frame):
    global running

    if running:
        update_target()

        if not update_missile():
            running = False

        missile_traj.append(missile_pos.copy())

        mt = np.array(missile_traj)

        # Update trajectory line
        traj_line.set_data(mt[:,0], mt[:,1])
        traj_line.set_3d_properties(mt[:,2])

        # Update missile and target dots
        missile_dot.set_data([missile_pos[0]], [missile_pos[1]])
        missile_dot.set_3d_properties([missile_pos[2]])

        target_dot.set_data([target_pos[0]], [target_pos[1]])
        target_dot.set_3d_properties([target_pos[2]])

    return missile_dot, target_dot, traj_line

ani = FuncAnimation(fig, update, interval=30, cache_frame_data=False)

plt.show()
