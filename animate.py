import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_trajectory(x_opt, y_opt, psi_opt, target_pos, obs, dt, left_thruster_opt, right_thruster_opt, u_opt, v_opt, r_opt):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot static obstacles
    for obstacle in obs:
        circle = plt.Circle((obstacle['x'], obstacle['y']), obstacle['radius'], color='orange', alpha=0.5, label='Obstacle')
        ax.add_patch(circle)

    # Plot target position
    ax.scatter(target_pos[0], target_pos[1], color='red', s=100, label='Target Position')

    # Initialize vessel marker
    vessel, = ax.plot([], [], 'bo-', lw=2, label='Vessel')
    yaw_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='blue')
    surge_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=12, color='green')
    thruster_text = ax.text(0.02, 0.85, '', transform=ax.transAxes, fontsize=12, color='purple')

    # Set axis limits based on trajectory
    ax.set_xlim(min(x_opt) - 10, max(x_opt) + 10)
    ax.set_ylim(min(y_opt) - 10, max(y_opt) + 10)
    ax.set_title('NMPC Vessel Trajectory Animation')
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.legend()
    ax.grid(True)


    arrow = ax.annotate('', xy=(0, 0), xytext=(0, 0), arrowprops=dict(facecolor='red', headwidth=6, headlength=8))


    def update(frame):
        if np.linalg.norm([x_opt[frame] - target_pos[0], y_opt[frame] - target_pos[1]]) < 1.0:
            anim.event_source.stop()
        vessel.set_data(x_opt[:frame + 1], y_opt[:frame + 1])
        surge_text.set_text(f'Surge Velocity: {u_opt[frame]:.2f} m/s')
        thruster_text.set_text(f'Thruster Commands: L={left_thruster_opt[frame]:.2f} N, R={right_thruster_opt[frame]:.2f} N')
        yaw_text.set_text(f'Yaw Angle: {psi_opt[frame]:.2f} rad')

        #Update arrow direction
        arrow_length = 2
        dx = arrow_length * np.cos(psi_opt[frame])
        dy = arrow_length * np.sin(psi_opt[frame])
        arrow.set_position((x_opt[frame], y_opt[frame]))
        arrow.xy = (x_opt[frame] + dx, y_opt[frame] + dy)

        return vessel, yaw_text, surge_text, thruster_text, arrow

    anim = FuncAnimation(fig, update, frames=len(x_opt), interval=dt * 1000, blit=True)
    plt.show()

    # Generate time vector
    N_steps = len(x_opt) - 1
    time_states = np.linspace(0, N_steps * dt, N_steps + 1)
    time_controls = np.linspace(0, (N_steps - 1) * dt, N_steps)

    # Plot states and control inputs over time
    plt.figure(figsize=(14, 12))

    # State plots
    plt.subplot(4, 2, 1)
    plt.plot(time_states, x_opt, '-b')
    plt.title('X Position [m]')
    plt.xlabel('Time [s]')
    plt.ylabel('X [m]')
    plt.grid(True)

    plt.subplot(4, 2, 2)
    plt.plot(time_states, y_opt, '-g')
    plt.title('Y Position [m]')
    plt.xlabel('Time [s]')
    plt.ylabel('Y [m]')
    plt.grid(True)

    plt.subplot(4, 2, 3)
    plt.plot(time_states, psi_opt, '-r')
    plt.title('Yaw Angle ψ [rad]')
    plt.xlabel('Time [s]')
    plt.ylabel('ψ [rad]')
    plt.grid(True)

    plt.subplot(4, 2, 4)
    plt.plot(time_states, u_opt, '-m')
    plt.title('Surge Velocity u [m/s]')
    plt.xlabel('Time [s]')
    plt.ylabel('u [m/s]')
    plt.grid(True)

    plt.subplot(4, 2, 5)
    plt.plot(time_states, v_opt, '-c')
    plt.title('Sway Velocity v [m/s]')
    plt.xlabel('Time [s]')
    plt.ylabel('v [m/s]')
    plt.grid(True)

    plt.subplot(4, 2, 6)
    plt.plot(time_states, r_opt, '-k')
    plt.title('Yaw Rate r [rad/s]')
    plt.xlabel('Time [s]')
    plt.ylabel('r [rad/s]')
    plt.grid(True)

    # Thruster commands
    plt.subplot(4, 2, 7)
    plt.plot(time_controls, left_thruster_opt, '-b')
    plt.title('Left Thruster Command')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.grid(True)

    plt.subplot(4, 2, 8)
    plt.plot(time_controls, right_thruster_opt, '-g')
    plt.title('Right Thruster Command')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.grid(True)

    plt.tight_layout()
    plt.show()