import datetime
import math
import numpy as np
import os
import pytz

from amr_localization.map import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        sensors: list[tuple[float, float, float]],
        sensor_range: float,
        particle_count: int,
        sigma_v: float = 0.15,
        sigma_w: float = 0.75,
        sigma_z: float = 0.25,
        logger=None,
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            sensors: Robot sensors' pose in the robot coordinate frame (x, y, theta) [m, m, rad].
            sensor_range: Sensor measurement range [m].
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._particle_count: int = particle_count
        self._sensors: list[tuple[float, float, float]] = sensors
        self._sensor_range: float = sensor_range
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._iteration: int = 0
        self._logger = logger

        self._map = Map(map_path, sensor_range, compiled_intersect=True, use_regions=True)
        self._particles = self._init_particles(particle_count)
        self._ds, self._phi = self._init_sensor_polar_coordinates(sensors)
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 2.10. Complete the missing function body with your code.
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))
        coordinates = self._particles[:, :2]
        #min_samples, 10. Eps is the distance between two samples.
        clustering = DBSCAN(eps=0.5, min_samples=10).fit(coordinates)
        labels = clustering.labels_
        n_clusters=len(set(labels)) - (1 if -1 in labels else 0)
        #Particle count adaptation
        if n_clusters > 1:
            self._particle_count = max(500,self._particle_count - 500/n_clusters )
        elif n_clusters == 1:
            self._particle_count= 100
            localized = True
        if localized:
            mask = (labels==0)
          
            cluster_points= self._particles[mask].astype(np.float64)
            mean_x= np.mean(cluster_points[:,0])
            mean_y= np.mean(cluster_points[:,1])
            sin_avg = np.mean(np.sin(cluster_points[:, 2]))
            cos_avg = np.mean(np.cos(cluster_points[:, 2]))
            mean_theta = np.arctan2(sin_avg, cos_avg)
            pose= (float(mean_x), float(mean_y), float(mean_theta))

        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 2.5. Complete the function body with your code (i.e., replace the pass statement).
        for i in range(len(self._particles)):
            x, y, theta = self._particles[i]
            x_prev=x
            y_prev=y
            v_noise= v + np.random.normal(0, self._sigma_v)
            w_noise= w + np.random.normal(0, self._sigma_v)
            x += v_noise * math.cos(theta) * self._dt 
            y += v_noise * math.sin(theta) * self._dt 
            theta += w_noise * self._dt 
            theta = theta % (2 * math.pi)
            res = self._map.check_collision([(x_prev, y_prev),(x, y)],True)
            if res[0]: 
                (x_int, y_int), dist = res
                self._particles[i]= (x_int, y_int, theta)
            else:
                self._particles[i] = (x, y, theta)
    
        
    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 2.9. Complete the function body with your code (i.e., replace the pass statement).
        probabilities = np.empty(len(self._particles))
        for i in range(len(self._particles)):
            probabilities[i] = self._measurement_probability(measurements, self._particles[i])

        # Normalize probabilities
        if np.sum(probabilities) > 0:
            probabilities /= np.sum(probabilities)
        elif np.sum(probabilities) == 0:
            probabilities= np.ones(len(self._particles))/len(self._particles)

        # Resample particles based on probabilities
        indices = np.random.choice(len(self._particles), size=len(self._particles), p=probabilities)
        self._particles = self._particles[indices]
        


        
    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(self, particle_count: int) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=object)
        x_min, y_min, x_max, y_max = self._map.bounds()

        self._logger.info(f"Initializing {particle_count} particles...")

        # TODO: 2.4. Complete the missing function body with your code.
        for i in range(particle_count):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)

            while not self._map.contains((x,y)):
                x = np.random.uniform(x_min, x_max)
                y = np.random.uniform(y_min, y_max)
                
            theta = np.random.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
            particles[i] = (x, y, theta)
           
        return particles


    @staticmethod
    def _init_sensor_polar_coordinates(
        sensors: list[tuple[float, float, float]],
    ) -> tuple[list[float], list[float]]:
        """Converts the sensors' poses to polar coordinates wrt to the robot's coordinate frame.

        Args:
            sensors: Robot sensors location and orientation (x, y, theta) [m, m, rad].

        Return:
            ds: List of magnitudes [m].
            phi: List of angles [rad].

        """
        ds = [math.sqrt(sensor[0] ** 2 + sensor[1] ** 2) for sensor in sensors]
        phi = [math.atan2(sensor[1], sensor[0]) for sensor in sensors]

        return ds, phi

    def _sense(self, particle: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every sensor given the robot's pose.

        Args:
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; inf if a sensor is out of range.

        """
        rays: list[list[tuple[float, float]]] = self._sensor_rays(particle)
        z_hat: list[float] = []

        # TODO: 2.6. Complete the missing function body with your code.
        for ray in rays:
            res = self._map.check_collision(ray, True)
            if res[0]: 
                _, dist = res
                z_hat.append(dist)
            else:
                z_hat.append(float("inf"))
        return z_hat
    

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 2.7. Complete the function body (i.e., replace the code below).
        return 1/(math.sqrt(2*math.pi)*sigma)*math.exp(-0.5*((x-mu)/sigma)**2)

        
    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        1.25 times the sensor range to perform the computation. This value has experimentally been
        proven valid to deal with missing measurements. Nevertheless, it might not be the optimal
        replacement value.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0
        z_hat = self._sense(particle)
        # TODO: 2.8. Complete the missing function body with your code.
        for meas, z_h in zip(measurements, z_hat):
        # Handle out-of-range for real measurement
            if math.isinf(meas):
                meas = 1.25 * self._sensor_range
                
            # Handle out-of-range for expected measurement
            if math.isinf(z_h):
                z_h = 1.25 * self._sensor_range
                
            # 3. Update probability (Product of Gaussians)
            probability *= self._gaussian(z_h, self._sigma_z, meas)
        
        return probability
        pass

    def _sensor_rays(self, particle: tuple[float, float, float]) -> list[list[tuple[float, float]]]:
        """Determines the simulated sensor ray segments for a given particle.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: Ray segments. Format:
                 [[(x0_begin, y0_begin), (x0_end, y0_end)],
                  [(x1_begin, y1_begin), (x1_end, y1_end)],
                  ...]

        """
        x = particle[0]
        y = particle[1]
        theta = particle[2]

        # Convert sensors to world coordinates
        xw = [x + ds * math.cos(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        yw = [y + ds * math.sin(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        tw = [sensor[2] for sensor in self._sensors]

        rays = []

        for xs, ys, ts in zip(xw, yw, tw):
            x_end = xs + self._sensor_range * math.cos(theta + ts)
            y_end = ys + self._sensor_range * math.sin(theta + ts)
            rays.append([(xs, ys), (x_end, y_end)])
        
        return rays