import numpy as np


#  Some important global variables, taken from tester.py
sensors_direction_dict = {'up': ['left', 'up', 'right'], 'right': ['up', 'right', 'down'],
                          'down': ['right', 'down', 'left'], 'left': ['down', 'left', 'up']}
movement_dict = {'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
opposite_direction_dict = {'up': 'down', 'right': 'left', 'down': 'up', 'left': 'right'}
#  delta = Left, Down, Right, Up
#  Taken from https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
#  Lesson 12, Quiz 17
delta = [[-1, 0], [0, -1], [1, 0], [0, 1]]
delta_name = ['left', 'down', 'right', 'up']
#  Mapping degrees to rotation.
delta_rotation = {'up': 0, 'left': -90, 'right': 90, 'down': 180}

#  A matrix to convert input binary sequence for wall to decimal.
wall_conversion_matrix_dict = {'up': np.asmatrix([8, 1, 2, 4]),
                               'down': np.asmatrix([2, 4, 8, 1]),
                               'left': np.asmatrix([4, 8, 1, 2]),
                               'right': np.asmatrix([1, 2, 4, 8])
                               }

#  Integer value specifying a cell is a dead end.
DEAD_END = 1000

#  Mapping degree of rotation to indexes.
rotation_index_mapper_dict = {-90: 0, 270: 0, 0: 1, 360: 1, 90: 2, -270: 2}
