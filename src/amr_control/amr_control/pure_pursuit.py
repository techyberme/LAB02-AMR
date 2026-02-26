import math
class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(self, dt: float, lookahead_distance: float = 0.5):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].

        """
        self._dt: float = dt
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._last_idx = 0  # Store the progress state

    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.4. Complete the function body with your code (i.e., compute v and w).
        v = 0.6
        w = 0.0

        try:
            origin_idx = self._find_closest_point(x, y)[1]
            target_xy = self._find_target_point((x, y),origin_idx=origin_idx)
            
            beta = math.atan2(target_xy[1] - y, target_xy[0] - x)

            # v = 1.0 * (1-math.sin(beta-theta))
            w = 2*v*math.sin(beta-theta)/(self._lookahead_distance)
            return v,w
        except:
            return 0.0, 0.0


        
       

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value
        self._last_idx = 0 #reset idx

    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.2. Complete the function body (i.e., find closest_xy and closest_idx).
        closest_xy = (0.0, 0.0)
        closest_idx = 0
        distance_sq = float("inf")
        for i in range(len(self._path)):
            new_dist_sq = (self._path[i][0]-x)**2 + (self._path[i][1]-y)**2
            if new_dist_sq < distance_sq:
                distance_sq = new_dist_sq
                closest_idx = i

        self._last_idx = closest_idx
        closest_xy = self._path[closest_idx]
        return closest_xy, closest_idx
        
    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.3. Complete the function body with your code (i.e., determine target_xy).
        target_xy = (0.0, 0.0)
        look_ahead_sq = self._lookahead_distance ** 2
        for i in range(origin_idx, len(self._path)):
            dist_sq = (self._path[i][0]-origin_xy[0])**2 + (self._path[i][1]-origin_xy[1])**2
            if dist_sq >= look_ahead_sq:
                p1 = self._path[i-1]
                p2 = self._path[i]
                #Circular expression from p0
                ax = origin_xy[0]
                ay = origin_xy[1]
                if (p2[0] - p1[0]) != 0:
                    # Linear expression from P1 to P2
                    # y = mx +n
                    m = (p2[1] - p1[1])/(p2[0] - p1[0])
                    n = p2[1] - m * p2[0]
                    
                    b = -2*ax + 2 * m * (n-ay)
                    a = m**2 + 1
                    c = ax**2 + (n-ay)**2 - look_ahead_sq
                    delta = math.sqrt(b**2 - 4*a*c)
                    x1 = (-b + delta)/(2*a)
                    x2 = (-b - delta)/(2*a)
                    # Take the nearest point to p2
                    target_x = x1 if abs(x1 - p2[0]) < abs(x2 - p2[0]) else x2
                    return (target_x, target_x * m + n)
                else: 
                    target_x = p1[0]
                    delta = math.sqrt(look_ahead_sq - (target_x -ax)**2) 
                    y1 = ay + delta
                    y2 = ay -delta
                    target_y = y1 if abs(y1 - p2[1]) < abs(y2 - p2[1]) else y2
                    return (target_x, target_y)
       


        return self._path[-1]
        