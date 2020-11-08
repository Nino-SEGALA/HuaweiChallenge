import numpy as np
from robot_explorers import Strategy

class Simple(Strategy):
    '''
    Attributes:
        directions (list): List of possible directions for all actions
        path (dict): Contains the trace of how each robot moved 
    '''

    def __init__(self):
        super().__init__()


    def init(self):
        self.map = []
        self.robot_state = {}
        self.print('num_robots: {}'.format(self.num_robots))
        self.print('shape: {}'.format(self.shape))
        self.print('max_energy: {}'.format(self.max_energy))
        self.print('coin_box (x,y): {}'.format(self.coin_box))
        self.path = {r:[] for r in range(self.num_robots)}

        for robot_id in range(self.num_robots):
            self.robot_state[robot_id] = {
                'detection_enabled': False,
                'using_radar': False,
                'directions_scanned': None
            }

        for i in range(self.shape[0]):
            new_row = []
            for j in range(self.shape[1]):
                new_row.append('X')
            self.map.append(new_row)


    def step(self, observation):
        action = self.action()

        # Loop over robots
        for robot_id in range(self.num_robots):
            action.detect(robot_id)

            if not self.robot_state[robot_id]['directions_scanned']:
                self.scan(robot_id, observation, 'right')
                self.robot_state[robot_id]['directions_scanned'] = 'right'
            elif self.robot_state[robot_id]['directions_scanned'] == 'right':
                self.scan(robot_id, observation, 'down')
                self.robot_state[robot_id]['directions_scanned'] = 'down'
            elif self.robot_state[robot_id]['directions_scanned'] == 'down':
                self.scan(robot_id, observation, 'left')
                self.robot_state[robot_id]['directions_scanned'] = 'left'
            elif self.robot_state[robot_id]['directions_scanned'] == 'left':
                self.scan(robot_id, observation, 'up')
                self.robot_state[robot_id]['directions_scanned'] = None

            
        return action

    def scan(self, robot_id, observation, direction):
        robot = observation.robot(robot_id)
        obj = observation.radar(robot_id, direction)
        
        right_x = left_x = top_y = bottom_y = None

        if direction == 'right':
            right_x = robot.position[0] + obj.distance
        elif direction == 'left':
            left_x = robot.position[0] - obj.distance
        elif direction == 'up':
            top_y = robot.position[1] - obj.distance
        elif direction == 'down':
            bottom_y = robot.position[1] + obj.distance
        
        self.print(obj)

        if obj.object == 'wall':
            if right_x:
                self.map[robot.position[1]][right_x] = 'W'
                for i in range(robot.position[0], right_x):
                    self.map[robot.position[1]][i] = 'O'
            if left_x:
                self.map[robot.position[1]][left_x] = 'W'
                for i in range(robot.position[0], left_x):
                    self.map[robot.position[1]][i] = 'O'
            if top_y:
                self.map[robot.position[0]][top_y] = 'W'
                for i in range(robot.position[1], top_y):
                    self.map[robot.position[0]][i] = 'O'
            if bottom_y:
                self.map[robot.position[0]][bottom_y] = 'W'
                for i in range(robot.position[1], bottom_y):
                    self.map[robot.position[0]][i] = 'O'
        elif obj.object == 'coin':
            if right_x:
                self.map[robot.position[1]][right_x] = 'C'
                for i in range(robot.position[0], right_x):
                    self.map[robot.position[1]][i] = 'O'
            if left_x:
                self.map[robot.position[1]][left_x] = 'C'
                for i in range(robot.position[0], left_x):
                    self.map[robot.position[1]][i] = 'O'
            if top_y:
                self.map[robot.position[0]][top_y] = 'C'
                for i in range(robot.position[1], top_y):
                    self.map[robot.position[0]][i] = 'O'
            if bottom_y:
                self.map[robot.position[0]][bottom_y] = 'C'
                for i in range(robot.position[1], bottom_y):
                    self.map[robot.position[0]][i] = 'O'
        elif obj.object == 'home_base':
            if right_x:
                self.map[robot.position[1]][right_x] = 'H'
                for i in range(robot.position[0], right_x):
                    self.map[robot.position[1]][i] = 'O'
            if left_x:
                self.map[robot.position[1]][left_x] = 'H'
                for i in range(robot.position[0], left_x):
                    self.map[robot.position[1]][i] = 'O'
            if top_y:
                self.map[robot.position[0]][top_y] = 'H'
                for i in range(robot.position[1], top_y):
                    self.map[robot.position[0]][i] = 'O'
            if bottom_y:
                self.map[robot.position[0]][bottom_y] = 'H'
                for i in range(robot.position[1], bottom_y):
                    self.map[robot.position[0]][i] = 'O'

        self.print(self.map)



# Run strategy
if __name__ == "__main__":
    strategy = Simple()
    strategy.run()




"""
[
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'H', 'O', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'H', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'H', 'O', 'O', 'O', 'O', 'O', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'H', 'O', 'O', 'O', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
    ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X']]
"""