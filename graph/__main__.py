from threading import Thread
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import re
from typing import List, Generator, TypedDict

FIELD_WIDTH = 144  # Field width in inches (assumes a square field)
SCALE = 24  # Scaling factor

class Point(TypedDict):
    x: float
    y: float

class Particle(Point):
    weight: float

class Pose(Point):
    theta: float

class Visualizer:
    def __init__(self):
        self.particles = []
        self.original_odometry = None
        self.mcl_odometry = None

        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [], c='red', label='Particles')
        self.original_odom_plot, = self.ax.plot([], [], 'bo', label='Original Odometry')
        self.mcl_odom_plot, = self.ax.plot([], [], 'go', label='MCL Odometry')

        # Set axis limits to center the origin (0,0) in the middle of the field
        self.ax.set_xlim(-FIELD_WIDTH / 2, FIELD_WIDTH / 2)
        self.ax.set_ylim(-FIELD_WIDTH / 2, FIELD_WIDTH / 2)
        
        # Set grid lines at intervals of 12
        self.ax.set_xticks(range(-FIELD_WIDTH // 2, FIELD_WIDTH // 2 + 1, 12))
        self.ax.set_yticks(range(-FIELD_WIDTH // 2, FIELD_WIDTH // 2 + 1, 12))
        self.ax.grid(True)
        
        self.ax.legend()

    def update_particles(self, particles: Generator[Particle, None, None]):
        self.particles = list(particles)

    def update_original_odometry(self, pose: Pose):
        self.original_odometry = pose

    def update_mcl_odometry(self, pose: Pose):
        self.mcl_odometry = pose

    def animate(self, i):
        if self.particles:
            x = [p['x'] / SCALE for p in self.particles]
            y = [p['y'] / SCALE for p in self.particles]
            self.scatter.set_offsets(list(zip(x, y)))

        if self.original_odometry:
            self.original_odom_plot.set_data([self.original_odometry['x'] / SCALE], [self.original_odometry['y'] / SCALE])

        if self.mcl_odometry:
            self.mcl_odom_plot.set_data([self.mcl_odometry['x'] / SCALE], [self.mcl_odometry['y'] / SCALE])

        return self.scatter, self.original_odom_plot, self.mcl_odom_plot

    def show(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100)
        plt.show()

def parse_pose(dataStr: str) -> Pose:
    # dataStr is formatted like: "<x_float>,<y_float>,<theta_float>"
    coords = dataStr.split(",")
    if len(coords) != 3:
        raise ValueError("Invalid pose telemetry: ", dataStr)
    x_new, y_new, theta_new = coords
    return Pose(x=float(x_new), y=float(y_new), theta=float(theta_new))

def split_iter(string, sep):
    # from: https://stackoverflow.com/a/9770397
    # warning: does not yet work if sep is a lookahead like `(?=b)`
    if sep == '':
        return (c for c in string)
    else:
        return (_.group(1)
                for _ in re.finditer(f'(?:^|{sep})((?:(?!{sep}).)*)', string))

def parse_particles(dataStr: str) -> Generator[Particle, None, None]:
    # dataStr is formatted like:
    # "<x_float_1>,<y_float_1>,<weight_float_1>;<x_float_2>,<y_float_2>,<weight_float_2>;..."
    for particle in split_iter(dataStr, ";"):
        coords = particle.split(",")
        if len(coords) == 1 and coords[0].strip() == "":
            # Should be end of string
            continue
        if len(coords) != 3:
            raise ValueError("Invalid particle telemetry: ", particle)
        x_new, y_new, weight_new = coords
        yield Particle(x=float(x_new),
                       y=float(y_new),
                       weight=float(weight_new))

def read_telemetry(visualizer: Visualizer):
    for line in sys.stdin:
        if line.startswith("[TELEMETRY.PF.DATA]:"):
            dataStr = line[len("[TELEMETRY.PF.DATA]:"):].strip()
            particles = parse_particles(dataStr)
            visualizer.update_particles(particles)
        elif line.startswith("[TELEMETRY.ODOM.MCL]:"):
            dataStr = line[len("[TELEMETRY.ODOM.MCL]:"):].strip()
            pose = parse_pose(dataStr)
            visualizer.update_mcl_odometry(pose)
        elif line.startswith("[TELEMETRY.ODOM.ORIGINAL]:"):
            dataStr = line[len("[TELEMETRY.ODOM.ORIGINAL]:"):].strip()
            pose = parse_pose(dataStr)
            visualizer.update_original_odometry(pose)

if __name__ == "__main__":
    visualizer = Visualizer()
    telemetry_thread = Thread(target=read_telemetry, args=(visualizer,))
    telemetry_thread.start()
    visualizer.show()
    telemetry_thread.join()