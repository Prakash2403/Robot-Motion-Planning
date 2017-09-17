from globals import *


def update_heading(robot_obj, rotation, movement):
    """
     This method updates the direction the robot is facing and its location in the maze
    :param robot_obj: Object of robot class
    :param rotation: Rotation to get to next determined cell
    :param movement: Movement to get to next determined cell
    :return: None
    """
    robot_obj.heading = sensors_direction_dict[robot_obj.heading][rotation_index_mapper_dict[rotation]]
    robot_obj.location[0] += movement_dict[robot_obj.heading][0] * movement
    robot_obj.location[1] += movement_dict[robot_obj.heading][1] * movement


def decimal_wall_num(robot_obj, sensors, down):
    sensors.append(down)
    return wall_conversion_matrix_dict[robot_obj.heading] * \
           (np.asmatrix(sensors).transpose())


def binarize_sensor_data(sensors):
    return [1 if i > 0 else 0 for i in sensors]


def wall_mapper(robot_obj, sensors, down):
    """
    Creates a map of walls in the maze. Inspired by a research paper and udacity's video on
    A* practical implementation..

    Name of research paper: Design and Implementation of a Robot for Maze-Solving using Flood-Fill Algorithm
    by Ibrahim Elshamarka and Abu Bakar Sayuti Saman.

    Udacity A* video link: https://classroom.udacity.com/courses/cs373/lessons/48646841/concepts/487161640923

    The implementation slightly differs the method mentioned in research paper. Research paper's
    algorithm uses a matrix of shape (2*maze_dim - 1, 2*maze_dim - 1). This method uses a separate
    matrix of dimension (maze_dim, maze_dim).

    :param robot_obj: Object of robot class.
    :param sensors: List containing readings of all sensors.
    :param down: Binary representation for wall present in downward direction. Value is 0, if wall is present,
    1 otherwise.
    :return: A decimal number, which when converted to binary, gives location of walls for a given cell.
    """
    sensors = binarize_sensor_data(sensors)
    return decimal_wall_num(robot_obj, sensors, down)


def determine_next_move(robot_obj, x1, y1, sensors):
    """
    Determines next move based on current co-ordinates and sensor readings.
    :param robot_obj: Object of robot class.
    :param x1: Current X co-ordinate of robot
    :param y1: Current Y co-ordinate of robot
    :param sensors: List containing sensor readings
    :return: Returns rotation and movement for next move.
    """
    if sensors == [0, 0, 0]:
        robot_obj.traversal_grid[x1][y1] = DEAD_END
        movement = -1
        rotation = 0
    else:
        rotation, movement = find_optimal_move(robot_obj, x1, y1, sensors)
    return rotation, movement


def find_optimal_move(robot_obj, x1, y1, sensors):
    """
    Some part of this method is taken from Udacity's Artificial Intelligence for Robotics course
    https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
    :param robot_obj: object of robot class
    :param x1: Current x coordinate
    :param y1: Current y coordinate
    :param sensors: Sensor readings
    :return: path to optimal cell which hasn't been explored yet.
    """

    possible_moves = []
    rotate = [-90, 0, 90]
    for sensor in range(len(sensors)):  # Look at all sensor readings
        if sensors[sensor] > 0:  # If it has a number, that means it's open.
            sensors[sensor] = 1

            #  Move to next open space
            x2 = x1 + movement_dict[sensors_direction_dict[robot_obj.heading][sensor]][0]
            y2 = y1 + movement_dict[sensors_direction_dict[robot_obj.heading][sensor]][1]

            #  Make sure robot next space is in maze
            if 0 <= x2 < robot_obj.maze_dim and 0 <= y2 < robot_obj.maze_dim:
                #  Preventing robot from entering into a path which leads to a dead end.
                if robot_obj.traversal_grid[x2][y2] != DEAD_END:
                    f_cost = robot_obj.traversal_grid[x2][y2]
                    h_cost = robot_obj.manhattan_heuristic_grid[x2][y2]
                    # Put elements in a list and append to possible_moves
                    # Elements are put in their priority order, i.e. while sorting
                    # list will be sorted w.r.t. f_cost(number of times a cell has been visited),
                    # If there is a tie, then we will sort them w.r.t. h_cost or heuristic value.
                    # Details are discussed in Project Report.
                    possible_moves.append([f_cost, h_cost, x2, y2, sensor])
                    #  Dead end path detection
                if robot_obj.traversal_grid[x2][y2] == DEAD_END and np.count_nonzero(sensors) == 1:
                    robot_obj.traversal_grid[x1][y1] = DEAD_END

    possible_moves.sort()
    possible_moves.reverse()
    try:
        min_move = possible_moves.pop()
    except IndexError:
        return 0, -1
    sensor = min_move[-1]
    return rotate[sensor], 1


def shortest_path(robot_obj, goal):
    #  Dynamic Programming algorithm
    #  Some of the code in this block is taken from  Udacity's AI for Robotics course
    #  https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
    #  Inspired by maze solving algorithm given on
    #  http://www.micromouseonline.com/micromouse-book/mazes-and-maze-solving/solving-the-maze/

    change = True  # Initialize boolean to start while-loop

    while change:
        change = False
        for x in range(robot_obj.maze_dim):
            for y in range(robot_obj.maze_dim):
                if goal[0] == x and goal[1] == y:
                    if robot_obj.path_value[x][y] > 0:
                        robot_obj.path_value[x][y] = 0
                        robot_obj.policy_grid[x][y] = '*'
                        print("Goal location: {}\n".format(goal))
                        change = True
                else:
                    wall = robot_obj.wall_map[x][y]
                    binary_wall = "{0:04b}".format(wall)
                    # Dynamic Programming. Discussed briefly in project report.
                    for direction in range(len(delta)):
                        if binary_wall[direction] == '1':
                            new_x_coordinate = x + delta[direction][0]
                            new_y_coordinate = y + delta[direction][1]
                            # Make sure inside maze
                            if 0 <= new_x_coordinate < robot_obj.maze_dim:
                                if 0 <= new_y_coordinate < robot_obj.maze_dim:
                                    temp_path_value = robot_obj.path_value[new_x_coordinate][new_y_coordinate] + 1
                                    if temp_path_value < robot_obj.path_value[x][y]:
                                        change = True
                                        robot_obj.path_value[x][y] = temp_path_value
                                        robot_obj.policy_grid[x][y] = delta_name[direction]
