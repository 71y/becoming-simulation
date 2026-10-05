"""
optimal variables:

dt = 0.1
c = 30
p = 0.3
k = 0
damping = 0.9

"""
from pylab import *
import random

import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

# All the variables.
frame_skip = 1  # fast forward
P = 0.3   # < 1, faster. 1 for true gravity/magnetism. > 1, slower.
K = 0
C = 10  # overall scaling factor
DAMPING = 0.9  # velocity damping factor/drag/friction
DT = 0.1


class InteractionPoint:
    # Random location between 0 and 100, random weight between 0 and 10
    def __init__(self, name):
        self.x = random.randrange(0, 100)
        self.y = random.randrange(0, 100)
        self.weight = random.randrange(1, 10)
        self.name = name

        self.radius = self.weight # size of the point for visualization

    def __str__(self):
        return f"InteractionPoint(name={self.name}, x={self.x}, y={self.y}, weight={self.weight}, radius={self.radius})"


def robot_derivative(current_robot_position):
    x, y = current_robot_position
    total_dx = 0.0
    total_dy = 0.0
    p = P   # < 1, faster. 1 for true gravity/magnetism. > 1, slower.
    k = K
    c = C
    for point in Interaction_Points:
        dx = point.x - x
        dy = point.y - y
        dist2 = dx * dx + dy * dy
        if dist2 < 0.01:
            dist2 = 0.01 # dont want to divide by zero
        pull = (c *  point.weight) / ((dist2 + k) ** p)
        r = np.sqrt(dist2) #normalize? the force in the x and y components
        if r != 0:
            fx = pull * dx / r
            fy = pull * dy / r
        else:
            fx = fy = 0
        total_dx += fx
        total_dy += fy
    return total_dx, total_dy


def robot_trajectory(dur, init_con, derivative):
    #duration of simulation
    #DT = 0.5
    number_of_iterations = int(dur // DT)
    print(f"Running simulation for {dur} seconds with time step of {DT} seconds ({number_of_iterations} iterations)")

    x0, y0 = init_con
    x, y = x0, y0
    velocity_dx, velocity_dy = 0, 0 # initial velocity
    t = 0

    robot_positions = [(x0, y0)]
    interaction_points_history = []
    consumed_frames = {}
    times = [0]

    damping = DAMPING  # velocity damping factor/drag/friction
    for it in range(number_of_iterations):
        points_to_remove = []
        for point in Interaction_Points:
            distance = np.sqrt((point.x - x)**2 + (point.y - y)**2)
            if distance < point.radius:
                points_to_remove.append(point)
        for point in points_to_remove:
            Interaction_Points.remove(point)
            interaction_points_history.append(point)
            consumed_frames[point.name] = len(robot_positions) - 1

        #calculate the acceleration based on the derivative function & calculate velocity using Euler's method
        acceleration_dx, acceleration_dy = derivative((x, y))
        velocity_dx = velocity_dx + acceleration_dx * DT
        velocity_dy = velocity_dy + acceleration_dy * DT

        # Apply velocity damping
        velocity_dx *= damping
        velocity_dy *= damping

        #Integrate position using Euler's method (semi-implicit)
        next_x = x + velocity_dx * DT
        next_y = y + velocity_dy * DT

        t = t + DT

        # keep track
        robot_positions.append((next_x, next_y))
        times.append(t)

        x = next_x
        y = next_y
    return robot_positions, times, interaction_points_history, consumed_frames


###################### Euler Integration ####################
# setting up the simulation by placing interaction points
Interaction_Points = [InteractionPoint(i) for i in range(10)]
"""
#testing with specific points
Interaction_Points[0].x = 54
Interaction_Points[0].y = 56
Interaction_Points[0].weight = 4
Interaction_Points[0].radius = 4

Interaction_Points[1].x = 89
Interaction_Points[1].y = 17
Interaction_Points[1].weight = 3
Interaction_Points[1].radius = 3

Interaction_Points[2].x = 8
Interaction_Points[2].y = 4
Interaction_Points[2].weight = 6
Interaction_Points[2].radius = 6

Interaction_Points[3].x = 56
Interaction_Points[3].y = 22
Interaction_Points[3].weight = 2
Interaction_Points[3].radius = 2

Interaction_Points[4].x = 88
Interaction_Points[4].y = 67
Interaction_Points[4].weight = 4
Interaction_Points[4].radius = 4

Interaction_Points[5].x = 23
Interaction_Points[5].y = 63
Interaction_Points[5].weight = 3
Interaction_Points[5].radius = 3

Interaction_Points[6].x = 37
Interaction_Points[6].y = 30
Interaction_Points[6].weight = 4
Interaction_Points[6].radius = 4

Interaction_Points[7].x = 36
Interaction_Points[7].y = 66
Interaction_Points[7].weight = 8
Interaction_Points[7].radius = 8

Interaction_Points[8].x = 37
Interaction_Points[8].y = 97
Interaction_Points[8].weight = 8
Interaction_Points[8].radius = 8

Interaction_Points[9].x = 15
Interaction_Points[9].y = 55
Interaction_Points[9].weight = 5
Interaction_Points[9].radius = 5

"""

Original_Interaction_Points = Interaction_Points.copy()
print("Interaction Points:")
for point in Interaction_Points:
    print(point)



# computing the trajectory of the robot given the initial conditions and the derivative function
robot_positions, times, interaction_points_history, consumed_frames = robot_trajectory(dur=5000, init_con=(50, 50), derivative=robot_derivative)
print([str(interaction_points_history[i].name) + f" (w={interaction_points_history[i].weight})" for i in range(len(interaction_points_history))], str(len(interaction_points_history)) + "/" + str(len(Original_Interaction_Points)))
# Extract x and y coordinates for animation
xs = [pos[0] for pos in robot_positions]
ys = [pos[1] for pos in robot_positions]

# compute velocities at each step
velocities = [(0, 0)]
for i in range(1, len(xs)):
    dx = xs[i] - xs[i-1]
    dy = ys[i] - ys[i-1]
    v = (dx/DT, dy/DT)
    velocities.append(v)



###################### Visualization ####################
# visualization of the simulation results using matplotlib animation
fig, ax = plt.subplots(figsize=(8, 8))

# points are scaled up for better visability
point_sizes = [(2*point.radius)**2 for point in Original_Interaction_Points]
# Overlay true-radius circles for each point and add labels
point_circles = []
point_labels = []
for i, point in enumerate(Original_Interaction_Points):
    circle = plt.Circle((point.x, point.y), point.radius, color='red', fill=False, alpha=0.5, linewidth=2)
    ax.add_patch(circle)
    point_circles.append(circle)
    # label with index and weight (data coordinates)
    label = ax.text(point.x + 1, point.y + 1, f"P{i} (w={point.weight})", fontsize=8, color='darkred', ha='left', va='bottom')
    point_labels.append(label)
# Scatter plot for the centers of the interaction points
point_scat = ax.scatter([point.x for point in Original_Interaction_Points], [point.y for point in Original_Interaction_Points], s=point_sizes, c="red", label="Interaction Points")

# Add live velocity counter
velocity_text = ax.text(0.5, -0.10, '', transform=ax.transAxes, ha='center', va='top', fontsize=12)

# stylzing the plot
line_traj, = ax.plot([], [], c="blue", lw=2, label="Robot Trajectory")
robot_marker, = ax.plot([], [], marker="o", c="black", markersize=8, label="Robot")

ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Robot trajectory toward interaction points")
ax.legend()




def update(frame):
    # Update trajectory line and robot marker
    line_traj.set_data(xs[:frame+1], ys[:frame+1])
    robot_marker.set_data([xs[frame]], [ys[frame]])
    # Update current velocity text
    vx, vy = velocities[frame]
    speed = np.sqrt(vx**2 + vy**2)
    velocity_text.set_text(f"Velocity: ({vx:.2f}, {vy:.2f}) | Total Speed: {speed:.2f}")

    point_colors = [
        "gray" if consumed_frames.get(point.name, float("inf")) <= frame else "red"
        for point in Original_Interaction_Points
    ]
    point_scat.set_color(point_colors)

    for point, circle, label, color in zip(Original_Interaction_Points, point_circles, point_labels, point_colors):
        circle.set_edgecolor(color)
        circle.set_alpha(0.35 if color == "gray" else 0.5)
        label.set_color("dimgray" if color == "gray" else "darkred")

    return line_traj, robot_marker, velocity_text, point_scat, *point_circles, *point_labels


ani = animation.FuncAnimation(fig=fig, func=update, frames=range(0, len(xs), frame_skip), interval=60, blit=False)
plt.show()
