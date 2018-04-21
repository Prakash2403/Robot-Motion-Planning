from globals import *
import utils


def optimization_trial(robot_obj):
    robot_obj.step_count += 1
    movement = 1
    # Retrieve current location
    curr_x_coordinate, curr_y_coordinate = robot_obj.location
    # Rotate to the optimal path
    heading = delta_rotation[robot_obj.heading]
    optimal_heading = delta_rotation[robot_obj.policy_grid[curr_x_coordinate][curr_y_coordinate]]
    rotation = optimal_heading - heading
    # Equivalent Rotation for 270 degree
    if rotation == -270:
        rotation = 90
    elif rotation == 270:
        rotation = -90
    direction = sensors_direction_dict[robot_obj.heading][rotation // 90 + 1]  # Change direction

    # Move up to 3 consecutive steps
    while movement < 3:  # Limit movement to 3 spaces
        location = robot_obj.policy_grid[curr_x_coordinate][curr_y_coordinate]
        curr_x_coordinate += movement_dict[direction][0]
        curr_y_coordinate += movement_dict[direction][1]

        if robot_obj.policy_grid[curr_x_coordinate][curr_y_coordinate] == location:
            movement += 1
        else:
            break

    # Update direction robot is facing & location
    utils.update_heading(robot_obj, rotation, movement)
    return rotation, movement
