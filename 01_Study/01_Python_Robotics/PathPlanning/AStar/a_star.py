"""

A* grid planning

author: Semin Chun

reference       :   Atsushi Sakai(@Atsushi_twi)
                    Nikos Kanargias (nkana@tee.gr)
reference git   :   https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py          

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

"""
    [Parameter]             [description] 
    f_Start_Px              : Start pont X [m]
    f_Start_Py              : Start pont Y [m]
    f_Goal_Px               : Goal pont X [m]
    f_Goal_Py               : Goal pont Y [m]
    f_Grid_size             : Resolution of Axis  # [m]
    f_Robot_radius          : robot radius [m]
    arry_f_Obstacle_PX      : Obstacle ponts X
    arry_f_Obstacle_Py      : Obstacle ponts y
"""


import math

import matplotlib.pyplot as plt

show_animation = True

class AStarPlanner:
    
    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y, self.max_x, self.max_y,\
        self.x_width, self.y_width, self.obstacle_map = self.get_obstacle_map_information(ox, oy, resolution)
        self.motion = self.get_motion_model()
        
    # Node information
    class Node:
        
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        goal_node = self.Node(self.Calc_Node_Index_By_Axis(gx, self.min_x),
                              self.Calc_Node_Index_By_Axis(gy, self.min_y), 0.0, -1)
        start_node = self.Node(self.Calc_Node_Index_By_Axis(sx, self.min_x),
                               self.Calc_Node_Index_By_Axis(sy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node, self.min_x, self.min_y, self.x_width)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            
            # find candidate which represents the minimun distance
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node, self.min_x, self.min_y, self.x_width)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node


        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
        
    # Calc_Node_Index_By_Axis (ex : x axis, y axis)
    def Calc_Node_Index_By_Axis (self, current_point, base_point):
        return round((current_point - base_point) / self.resolution)

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos 
        
    def get_obstacle_map_information(self, ox, oy, resolution):
        """
        get_obstacle_map_information
        
        Input
        ox          : x ponints list of obstacle
        oy          : y ponints list of obstacle
        resolution  : resolution of vehicle motion
        
        Output
        min_x       : minimum boundary which is x ponint in X-axis on Map
        min_y       : minimum boundary which is y ponint in Y-axis on Map
        max_x       : maximum boundary which is x ponint in X-axis on Map
        max_y       : maximum boundary which is y ponint in Y-axis on Map
        x_width     : the number of the grid of x-axis -1
        y_width     : the number of the grid of y-axis -1
        obstacle_map: this map represent the obstacle resion. true is in obstacle resion, otherwise false.
        """
        min_x = round(min(ox))
        min_y = round(min(oy))
        max_x = round(max(ox))
        max_y = round(max(oy))
        print("min_x:", min_x)
        print("min_y:", min_y)
        print("max_x:", max_x)
        print("max_y:", max_y)

        x_width = round((max_x - min_x) / resolution)
        y_width = round((max_y - min_y) / resolution)
        print("x_width:", x_width)
        print("y_width:", y_width)

        # obstacle map generation
        obstacle_map = [[False for _ in range(y_width)]
                             for _ in range(x_width)]
        for ix in range(x_width):
            x = self.calc_grid_position(ix, min_x)
            for iy in range(y_width):
                y = self.calc_grid_position(iy, min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        obstacle_map[ix][iy] = True
                        break
                    
        return min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map

    def calc_grid_index(self, node, min_x, min_y, x_width):
        return (node.y - min_y) * x_width + (node.x - min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry
                 
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

            

def main() :
    print(__file__ + " start!!")
    
    # start and goal position
    f_Start_Px = 10.0  # [m]
    f_Start_Py = 10.0  # [m]
    f_Goal_Px = 50.0  # [m]
    f_Goal_Py = 50.0  # [m]
    f_Grid_size = 2.0  # [m]
    f_Robot_radius = 1.0  # [m]
    
    # set obstacle positions
    arry_f_Obstacle_PX, arry_f_Obstacle_Py = [], []
    for i in range(-10, 60):
        arry_f_Obstacle_PX.append(i)
        arry_f_Obstacle_Py.append(-10.0)
    for i in range(-10, 60):
        arry_f_Obstacle_PX.append(60.0)
        arry_f_Obstacle_Py.append(i)
    for i in range(-10, 61):
        arry_f_Obstacle_PX.append(i)
        arry_f_Obstacle_Py.append(60.0)
    for i in range(-10, 61):
        arry_f_Obstacle_PX.append(-10.0)
        arry_f_Obstacle_Py.append(i)
    for i in range(-10, 40):
        arry_f_Obstacle_PX.append(20.0)
        arry_f_Obstacle_Py.append(i)
    for i in range(0, 40):
        arry_f_Obstacle_PX.append(40.0)
        arry_f_Obstacle_Py.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(arry_f_Obstacle_PX, arry_f_Obstacle_Py, ".k")
        plt.plot(f_Start_Px, f_Start_Py, "og")
        plt.plot(f_Goal_Px, f_Goal_Py, "xb")
        plt.grid(True)
        plt.axis("equal")
      
    a_star = AStarPlanner(arry_f_Obstacle_PX, arry_f_Obstacle_Py, f_Grid_size, f_Robot_radius)
    rx, ry = a_star.planning(f_Start_Px, f_Start_Py, f_Goal_Px, f_Goal_Py)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()        


if __name__ == '__main__':
    main()
