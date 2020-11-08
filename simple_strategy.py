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
#         self.map = [
#     ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'X', 'H', 'O', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'H', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'H', 'O', 'O', 'O', 'O', 'O', 'W', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'H', 'O', 'O', 'O', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'O', 'C', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
#     ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X']
# ]
        self.map = [
            ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'O', 'X', 'X', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'O', 'X', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'O', 'X', 'O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'X', 'X', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'X', 'X', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'X', 'X', 'X', 'X', 'X', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'], 
            ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X']
        ]
        self.get_path((1,1),(8,7))
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
                'directions_scanned': None,
                'priority_path' : [], # If robot has to go to a coin or come back to home base, we call get_path()
                'priority_path_index': None
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
                self.robot_state[robot_id]['directions_scanned'] = 'up'
            elif self.robot_state[robot_id]['directions_scanned'] == 'up':
                robot = observation.robot(2)
                self.print(robot.position)
                self.robot_state[robot_id]['directions_scanned'] = 'sdfdsf'
                self.get_path((robot.position), (4, 6))

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
                    self.map[i][robot.position[1]] = 'O'
                
    
        # self.print_map()
        # self.print()

    def print_map(self):
        for i in range(self.shape[0]):
            self.print(self.map[i])



    def get_path(self, start, end):
        a = self.map
        self.print_map()
        for i in range(len(a)):
            for j in range(len(a[i])):
                if a[i][j] == 'O':
                    a[i][j] = 0
                else:
                    a[i][j] = 1

        m = []
        for i in range(len(a)):
            m.append([])
            for j in range(len(a[i])):
                m[-1].append(0)
        i,j = start
        m[i][j] = 1

        k = 0
        while m[end[0]][end[1]] == 0:
            k += 1
            for i in range(len(m)):
                for j in range(len(m[i])):
                    if m[i][j] == k:
                        if i>0 and m[i-1][j] == 0 and a[i-1][j] == 0:
                            m[i-1][j] = k + 1
                        if j>0 and m[i][j-1] == 0 and a[i][j-1] == 0:
                            m[i][j-1] = k + 1
                        if i<len(m)-1 and m[i+1][j] == 0 and a[i+1][j] == 0:
                            m[i+1][j] = k + 1
                        if j<len(m[i])-1 and m[i][j+1] == 0 and a[i][j+1] == 0:
                            m[i][j+1] = k + 1
        
        i, j = end
        k = m[i][j]
        path = [(i,j)]
        while k > 1:
            if i > 0 and m[i - 1][j] == k-1:
                i, j = i-1, j
                path.append((i, j))
                k-=1
            elif j > 0 and m[i][j - 1] == k-1:
                i, j = i, j-1
                path.append((i, j))
                k-=1
            elif i < len(m) - 1 and m[i + 1][j] == k-1:
                i, j = i+1, j
                path.append((i, j))
                k-=1
            elif j < len(m[i]) - 1 and m[i][j + 1] == k-1:
                i, j = i, j+1
                path.append((i, j))
                k -= 1

        path.reverse()
        self.print(path)

        # Get Moves
        moves = []
        for i in range(len(path)-1):
            self.print(path[i],path[i+1])
            if path[i+1][0] > path[i][0]: # down
                moves.append('down')
            elif path[i+1][0] < path[i][0]: # up
                moves.append('up')
            elif path[i+1][1] > path[i][1]: # right
                moves.append('right')
            elif path[i+1][1] < path[i][1]: # left
                moves.append('left')

        self.print(moves)

        
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