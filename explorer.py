import numpy as np
import utils


def exploration_trial(robot_obj, sensors):
    robot_obj.step_count += 1
    # Print the robot's location
    curr_x_coordinate = robot_obj.location[0]
    curr_y_coordinate = robot_obj.location[1]
    # Add 1 to path_grid
    robot_obj.traversal_grid[curr_x_coordinate][curr_y_coordinate] += 1

    # Calculate the percentage of the maze the robot has visited
    path_count = np.count_nonzero(np.asmatrix(robot_obj.traversal_grid))
    discovered = (path_count / float((robot_obj.maze_dim ** 2))) * 100

    # Draw the map of the maze from the sensor readings
    num = utils.wall_mapper(robot_obj, sensors, robot_obj.down)
    robot_obj.wall_map[curr_x_coordinate][curr_y_coordinate] = num

    # Determine the robot's next move
    rotation, movement = utils.determine_next_move(robot_obj, curr_x_coordinate, curr_y_coordinate, sensors)
    if movement == 1:
        robot_obj.down = 1
    elif movement == -1:
        robot_obj.down = 0
    # Update the robot's direction it is facing & new location
    utils.update_heading(robot_obj, rotation, movement)
    # Get the new location
    new_x_coordinate = robot_obj.location[0]
    new_y_coordinate = robot_obj.location[1]
    # See if new location is in the goal.
    if new_x_coordinate in robot_obj.goals and new_y_coordinate in robot_obj.goals:
        if robot_obj.traversal_grid[new_x_coordinate][new_y_coordinate] == 0:
            if robot_obj.goal_discovered is False:
                print("Robot found the goal position after {} steps.".format(robot_obj.step_count))
                print("Continuing exploration........")
                robot_obj.goal_discovered = True

    elif robot_obj.goal_discovered and discovered >= robot_obj.exploration_percent:
        print("Robot has ended exploration after {} steps. Starting Optimization trial".format(robot_obj.step_count))
        utils.shortest_path(robot_obj, robot_obj.goals)  # Compute the value function and find optimal path
        print('Wall map')
        print(robot_obj.wall_map)
        print('Traversal Grid')
        print(robot_obj.traversal_grid)
        print('Path Value')
        print(np.asmatrix(robot_obj.path_value))
        # Restore to default settings and start Optimization Trial
        rotation = 'Reset'
        movement = 'Reset'
        robot_obj.optimization_run = True
        robot_obj.reset()
    return rotation, movement
