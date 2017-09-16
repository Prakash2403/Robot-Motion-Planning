from __future__ import print_function

import numpy as np
import explorer
import optimizer


class Robot(object):
    def __init__(self, maze_dim):
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        #  Map of walls in grid, explored by robot.
        #  Walls are stored in a way similar to test cases.
        self.wall_map = np.zeros((maze_dim, maze_dim), dtype=np.int)

        #  This variable is True when goal is discovered, yet we want to continue exploration
        #  to find some other path, which might be optimal.
        self.goal_discovered = False

        #  Exploration percent is percentage of map to be explored before exploration stops.
        #  In this case, Exploration will stop when goal is discovered and map is 50% explored.
        self.exploration_percent = 60

        #  Stores number of times a cell has been traversed. Obviously, initial value is 0.
        self.traversal_grid = np.zeros((maze_dim, maze_dim), dtype=np.int)

        #  Taken from Udacity AI for robotics course
        #  Quiz Link: https://classroom.udacity.com/courses/cs373/lessons/48646841/concepts/485327600923
        self.path_value = [[99 for _ in range(maze_dim)] for _ in range(maze_dim)]

        #  Inspired by Udacity AI for robotics course
        self.manhattan_heuristic_grid = [[min(abs(row - maze_dim / 2 + 1), abs(row - maze_dim / 2)) +
                                          min(abs(col - maze_dim / 2 + 1),
                                              abs(col - maze_dim / 2)) for
                                          row in range(maze_dim)] for col in range(maze_dim)]  # Heuristic Grid
        #  Taken from Udacity AI for robotics course
        #  Quiz link: https://classroom.udacity.com/courses/cs373/lessons/48646841/concepts/485327600923
        self.policy_grid = [[' ' for _ in range(maze_dim)] for _ in range(maze_dim)]  # Grid to map optimal route.

        #  Possible goal locations. All 4 centers of grid.
        self.goals = [maze_dim / 2 - 1, maze_dim / 2] or \
                     [maze_dim / 2, maze_dim / 2] or \
                     [maze_dim / 2, maze_dim / 2 - 1] or \
                     [maze_dim / 2 - 1, maze_dim / 2 - 1]
        #  Binary representation for presence of wall in downward direction.
        #  Can take only 2 values, 0 and 1. 0 if wall is present, 1 otherwise.
        self.down = 0
        self.step_count = 0  # Number of steps taken in each trial

        #  This variable is False when exploration trial is running, true otherwise.
        self.optimization_run = False

    def next_move(self, sensors):
        """
        Determines next moves based on sensor readings and optimization_run value
        :param sensors: List of left, forward and right sensor readings
        :return: Returns rotation and number of moves for next step.
        """
        if self.optimization_run:
            rotation, movement = optimizer.optimization_trial(self)
        else:
            rotation, movement = explorer.exploration_trial(self, sensors)
        return rotation, movement

    def reset(self):
        self.location = [0, 0]
        self.heading = 'up'
        self.step_count = 0
