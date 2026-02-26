import datetime
import math
import numpy as np
import os
import pytz

from amr_planning.map import Map
from matplotlib import pyplot as plt


class AStar:
    """Class to plan the optimal path to a given location using the A* algorithm."""

    def __init__(
        self,
        map_path: str,
        sensor_range: float,
        action_costs: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
    ):
        """A* class initializer.

        Args:
            map_path: Path to the map of the environment.
            sensor_range: Sensor measurement range [m].
            action_costs: Cost of of moving one cell left, right, up, and down.

        """
        self._actions: np.ndarray = np.array(
            [
                (-1, 0),  # Move one cell left
                (0, 1),  # Move one cell up
                (1, 0),  # Move one cell right
                (0, -1),  # Move one cell down
            ]
        )
        self._action_costs: tuple[float, float, float, float] = action_costs
        self._map: Map = Map(map_path, sensor_range, compiled_intersect=False, use_regions=False)

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def a_star(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> tuple[list[tuple[float, float]], int]:
        """Computes the optimal path to a given goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Destination in (x, y) format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.
            Number of A* iterations required to find the path.

        """
        # TODO: 3.2. Complete the function body (i.e., replace the code below).
        path: list[tuple[float, float]] = []
        steps: int = 0

        start_rc= self._xy_to_rc(start)
        goal_rc= self._xy_to_rc(goal)

        heuristic = self._compute_heuristic(goal)
        h_initial = heuristic[start_rc[0], start_rc[1]]

        g_initial = 0
        f_initial = h_initial + g_initial

        open_list = {(start_rc[0], start_rc[1]): (f_initial, g_initial)}
        closed_list = set()
        ancestors = {}
        parent = {start_rc: None}

        while open_list:
            steps +=1

            node = min(open_list, key =lambda k:open_list.get(k)[0])
            
            def compute_dir(start_node, step_node):
                x0, y0 = start_node
                x1, y1 = step_node
            
                if x0 == x1:
                    if y1>y0:
                        dir = 2
                    if y1<y0:
                        dir = 4
                if y0 == y1:
                    if x1>x0:
                        dir = 1
                    if x1<x0:
                        dir = 3
                else:
                    dir = None

                return dir

            g_value = open_list[node][1]
            open_list.pop(node)
            closed_list.add(node)

            if node == goal_rc:
                break 

            neighbours= [(node[0]+1, node[1]),   # dir: 1
                          (node[0], node[1]+1),  # dir: 2
                          (node[0]-1, node[1]),  # dir: 3
                          (node[0], node[1]-1)]  # dir: 4
            
            rows, cols = self._map.grid_map.shape
            
            prev = parent.get(node)
            prev_dir = compute_dir(prev, node) if prev is not None else None

            for neighbour in neighbours:
                r, c = neighbour
                
                dir = compute_dir(node, neighbour) 
                cost = 1 if dir == prev_dir else 2

                if r<0 or r>=rows or c<0 or c>=cols:
                    continue

                if not self._map.grid_map[neighbour[0]][neighbour[1]] and neighbour not in open_list and neighbour not in closed_list:
                    h = heuristic[neighbour[0], neighbour[1]]

                    cost = 1 if (prev_dir is None or dir == prev_dir) else 10                  

                    g_new = g_value + cost
                    open_list[neighbour] = (h + g_new, g_new)
                    ancestors[self._rc_to_xy(neighbour)] = self._rc_to_xy(node)
                    parent[neighbour] = node

        path= self._reconstruct_path(start,goal,ancestors)

        return path, steps

        
    @staticmethod
    def smooth_path(
        path, data_weight: float = 0.1, smooth_weight: float = 0.1, tolerance: float = 1e-6
    ) -> list[tuple[float, float]]:
        """Computes a smooth trajectory from a Manhattan-like path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                       less than this value.

        Returns: Smoothed path (initial location first) in (x, y) format.

        """
         # TODO: 3.4. Complete the function body (i.e., load smoothed_path).
        new_path = []
        for i in range(len(path)-1):
            x0, y0 = path[i]
            x1, y1 = path[i+1]

            new_path.append((x0,y0))

            for j in range(1,4):
                increment = j / 4
                x = x0 + increment*(x1-x0)
                y = y0 + increment*(y1-y0)
                new_path.append((x,y))
        new_path.append(path[-1])

        smoothed_path = new_path.copy()
        p = new_path.copy()

        error = math.inf

        while error > tolerance:
            error = 0
            s_new = smoothed_path.copy()
            for i in range(1,len(p)-1):
                change_x = data_weight*(p[i][0]-smoothed_path[i][0]) + smooth_weight*(smoothed_path[i+1][0] + smoothed_path[i-1][0] - 2*smoothed_path[i][0])

                change_y = data_weight*(p[i][1]-smoothed_path[i][1]) + smooth_weight*(smoothed_path[i+1][1] + s_new[i-1][1] - 2*s_new[i][1])

                s_new[i] = ((s_new[i][0] + change_x,s_new[i][1] + change_y))

                error += np.abs(change_x) + np.abs(change_y)
            smoothed_path = s_new
        return smoothed_path

    @staticmethod
    def plot(axes, path: list[tuple[float, float]], smoothed_path: list[tuple[float, float]] = ()):
        """Draws a path.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        x_val = [x[0] for x in path]
        y_val = [x[1] for x in path]

        axes.plot(x_val, y_val)  # Plot the path
        axes.plot(
            x_val[1:-1], y_val[1:-1], "bo", markersize=4
        )  # Draw blue circles in every intermediate cell

        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(
                x_val[1:-1], y_val[1:-1], "yo", markersize=4
            )  # Draw yellow circles in every intermediate cell

        axes.plot(x_val[0], y_val[0], "rs", markersize=7)  # Draw a red square at the start location
        axes.plot(
            x_val[-1], y_val[-1], "g*", markersize=12
        )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        path,
        smoothed_path=(),
        title: str = "Path",
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays a given path on the map.

        Args:
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
            title: Plot title.
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
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _compute_heuristic(self, goal: tuple[float, float]) -> np.ndarray:
        """Creates an admissible heuristic.

        Args:
            goal: Destination location in (x,y) coordinates.

        Returns:
            Admissible heuristic.
             Complete the method _compute_heuristic of the AStar class, which receives the goal
location in the environments coordinate frame (i.e., in (x, y) format), and returns an admissible
heuristic using the Manhattan distance. To obtain the size of the grid map you might want to use
numpy.shape

        """
    
        heuristic = np.zeros_like(self._map.grid_map)
        goal_rc= self._xy_to_rc(goal)

        H,W= np.shape(self._map.grid_map)
        for i in range(H):
            for j in range(W):
                heuristic[i][j]= np.abs(goal_rc[0]-i) + np.abs(goal_rc[1]-j)
        return heuristic

        
        

        # TODO: 3.1. Complete the missing function body with your code.
        
    
    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Goal location in (x, y) format.
            ancestors: Matrix that contains for every cell, None or the (x, y) ancestor from which
                       it was opened.

        Returns: Path to the goal (start location first) in (x, y) format.
        Fill in the missing code in _reconstruct_path to determine the optimal path from the
starting location as a set of points in world coordinates. You will also have to modify a_star slightly to
keep track of every cells ancestor (i.e. the cell you came from when you added a node to the open list).
To do so, create a new empty dictionary in the a_star method after defining the closed list, and every
time you insert a new node to the open list, add the origin node to the ancestors container too.

        """
        path: list[tuple[float, float]] = []
        # TODO: 3.3. Complete the missing function body with your code.
       
        current_node = goal
        
        # Añadimos el objetivo al final de la lista
        path.append(current_node)

        # Reconstruimos mientras el nodo actual tenga un padre registrado
        # start no estará en ancestors.keys(), así que el bucle parará ahí.
        while current_node in ancestors:
            current_node = ancestors[current_node]
            path.append(current_node)
        
        path.reverse()
        return path



        
      

    def _xy_to_rc(self, xy: tuple[float, float]) -> tuple[int, int]:
        """Converts (x, y) coordinates of a metric map to (row, col) coordinates of a grid map.

        Args:
            xy: (x, y) [m].

        Returns:
            rc: (row, col) starting from (0, 0) at the top left corner.

        """
        map_rows, map_cols = np.shape(self._map.grid_map)

        x = round(xy[0])
        y = round(xy[1])

        row = int(map_rows - (y + math.ceil(map_rows / 2.0)))
        col = int(x + math.floor(map_cols / 2.0))

        return row, col

    def _rc_to_xy(self, rc: tuple[int, int]) -> tuple[float, float]:
        """Converts (row, col) coordinates of a grid map to (x, y) coordinates of a metric map.

        Args:
            rc: (row, col) starting from (0, 0) at the top left corner.

        Returns:
            xy: (x, y) [m].

        """
        map_rows, map_cols = np.shape(self._map.grid_map)
        row, col = rc

        x = col - math.floor(map_cols / 2.0)
        y = map_rows - (row + math.ceil(map_rows / 2.0))

        return x, y
