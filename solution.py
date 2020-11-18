import numpy as np
from robot_explorers import Strategy
import random
import copy

class Simple(Strategy):
    '''
    Attributes:
        directions (list): List of possible directions for all actions
        path (dict): Contains the trace of how each robot moved 
    '''

    def __init__(self):
        super().__init__()


    def init(self):

        self.object_map = {
            'wall': 'W',
            'coin': 'C',
            'home_base': 'H',
            'robot': 'O'
        }

        self.opp_direction = {
            'left': 'right',
            'up': 'down',
            'down': 'up',
            'right': 'left'
        }

        self.direction_list = ['right', 'down', 'left', 'up']

        self.map = []
        self.has_scouts = False

        self.scout_id = 0

        self.robot_state = {}

        for robot_id in range(self.num_robots):
            self.robot_state[robot_id] = {
                'detection_enabled': False,
                'optimal_path_to_home' : [],
                'optimal_path_set': False,
                'priority_path_index': None,
                'role': 'W',
                'going_for_coin': False,
                'prev_move': None,
                'direction_idx': 0,
                'priority_directions': ['right', 'down', 'up', 'left']
            }

        for i in range(self.shape[0]):
            new_row = []
            for j in range(self.shape[1]):
                new_row.append('X')
            self.map.append(new_row)


    def goto_home(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        robot_state = self.robot_state[robot_id]

        if robot.has_item and not robot_state['optimal_path_set']:
            self.get_path(robot_id, robot.position, (1, 2))
            
        if robot.has_item and len(robot_state['optimal_path_to_home']) > 0:
            robot_state['prev_move'] = None
            robot_state['going_for_coin'] = False
            
            direction_to_move = robot_state['optimal_path_to_home'][0]
            
            self.print("WILL GO TOWARDS HOME BASE", direction_to_move, robot_id)
            if self.get_neighbor(robot, observation, direction_to_move) in ('O', 'H'):
                self.print("HOME BASE CALLING", direction_to_move, robot_id)
                robot_state['optimal_path_to_home'].pop(0)
                self.move(robot_id, observation, action, direction_to_move)
            
            return True
        
        return False

    def explore(self, robot_id, observation, action):
        robot_state = self.robot_state[robot_id]
        robot = observation.robot(robot_id)

        robot_state['optimal_path_set'] = False

        if not robot_state['going_for_coin']:
            if self.look_straight(robot, observation, 'right')[0] == 'X' or self.look_straight(robot, observation, 'down')[0] == 'X' or self.look_straight(robot, observation, 'left')[0] == 'X' or self.look_straight(robot, observation, 'up')[0] == 'X':
                self.print("SCANNING...")
                self.scan(robot_id, observation, action)
                return True
        
        return False

    def greedy(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        robot_state = self.robot_state[robot_id]

        priority_direction = self.get_direction_priority_when_coin_spot(robot, observation)
        
        if priority_direction:
            robot_state['going_for_coin'] = True
            if self.get_neighbor(robot, observation, priority_direction) not in ('O', 'H', 'C'):
                return True
            self.move(robot_id, observation, action, priority_direction)
            self.print("MOVING {} BECAUSE OF COIN".format(priority_direction), robot_id)
            
            if priority_direction == 'right':
                self.map[robot.position[1]][robot.position[0] + 1] = 'O'
            if priority_direction == 'left':
                self.map[robot.position[1]][robot.position[0] - 1] = 'O'
            if priority_direction == 'up':
                self.map[robot.position[1] - 1][robot.position[0]] = 'O'
            if priority_direction == 'down':
                self.map[robot.position[1] + 1][robot.position[0]] = 'O'
            return True
        robot_state['going_for_coin'] = False

        return False

    def go_free(self, robot_id, observation, action):
        robot_state = self.robot_state[robot_id]
        robot = observation.robot(robot_id)

        for direction in robot_state['priority_directions']:
            if self.get_neighbor(robot, observation, direction) == 'O':
                self.print("MOVING {}".format(direction), robot_id)
                self.move(robot_id, observation, action, direction)
                return True

    def move(self, robot_id, observation, action, direction):
        robot = observation.robot(robot_id)
        self.map[robot.position[1]][robot.position[0]] = 'O'
        action.move(robot_id, direction)

    def step(self, observation):
        action = self.action()

        for robot_id in range(self.num_robots):

            # Robot has coin, return to home base
            if self.goto_home(robot_id, observation, action):
                continue

            # Scan
            if self.explore(robot_id, observation, action):
                continue
            
            # If coins spotted, prioritize the coin with smallest distance and just go for it!
            if self.greedy(robot_id, observation, action):
                continue
            
            # Nothing to do, if neighbor is empty, take 1 move towards it
            self.go_free(robot_id, observation, action)
            
        return action

    def get_direction_priority_when_coin_spot(self, robot, observation):
        items = {
            'left': self.look_straight(robot, observation, 'left'), 
            'right': self.look_straight(robot, observation, 'right'), 
            'up': self.look_straight(robot, observation, 'up'), 
            'down': self.look_straight(robot, observation, 'down')
        }
        direction = None
        distance = 9999999
        for key, value in items.items():
            if value[0] == 'C' and value[1] <= distance:
                distance = value[1]
                direction = key

        return direction

    def move_down(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        if self.map[robot.position[1]+1][robot.position[0]] not in ('W', 'R', 'X'):
            self.map[robot.position[1]][robot.position[0]] = 'O'
            action.move(robot_id, 'down')
            return True

        return False
    
    def move_left(self, robot_id, observation, action):
        robot = observation.robot(robot_id)

        if self.map[robot.position[1]][robot.position[0]-1] not in ('W', 'R', 'X'):
            self.map[robot.position[1]][robot.position[0]] = 'O'
            action.move(robot_id, 'left')
            return True

        return False

    def move_right(self, robot_id, observation, action):
        robot = observation.robot(robot_id)

        if self.map[robot.position[1]][robot.position[0]+1] not in ('W', 'R', 'X'):
            self.map[robot.position[1]][robot.position[0]] = 'O'
            action.move(robot_id, 'right')
            return True

        return False
    
    def move_up(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        if self.map[robot.position[1]-1][robot.position[0]]  not in ('W', 'R', 'X'):
            self.map[robot.position[1]][robot.position[0]] = 'O'
            action.move(robot_id, 'up')
            return True
        return False

    def look_straight(self, robot, observation, direction):
        robot_x = robot.position[0]
        robot_y = robot.position[1]
        distance = 0

        robot_neighbor = self.check_own_robot(robot, observation)

        if direction == 'right':
            if not robot_neighbor['right']:
                return ('R', 1)
            while self.map[robot_y][robot_x + 1] == 'O':
                robot_x += 1
                distance += 1
            return self.map[robot_y][robot_x + 1], distance

        if direction == 'left':
            if not robot_neighbor['left']:
                return ('R', 1)
            while self.map[robot_y][robot_x - 1] == 'O':
                robot_x -= 1
                distance += 1
            return self.map[robot_y][robot_x - 1], distance

        if direction == 'up':
            if not robot_neighbor['up']:
                return ('R', 1)
            while self.map[robot_y - 1][robot_x] == 'O':
                robot_y -= 1
                distance += 1
            return self.map[robot_y - 1][robot_x], distance
        
        if direction == 'down':
            if not robot_neighbor['down']:
                return ('R', 1)
            while self.map[robot_y + 1][robot_x] == 'O':
                robot_y += 1
                distance += 1
            return self.map[robot_y + 1][robot_x], distance

    def check_own_robot(self, robot, observation):
        robot_position = robot.position
        free_direction = {
            'right': True,
            'down': True,
            'left': True,
            'up': True
        }

        for r_id in range(self.num_robots):
            r = observation.robot(r_id)
            if r.position == robot_position:
                continue

            if r.position[0] - 1 == robot_position[0] and r.position[1] == robot_position[1]:
                free_direction['right'] = False

            if r.position[0] + 1 == robot_position[0] and r.position[1] == robot_position[1]:
                free_direction['left'] = False

            if r.position[0] == robot_position[0] and r.position[1] - 1 == robot_position[1]:
                self.print("DOWN ROBOT", r, robot_position)
                free_direction['down'] = False

            if r.position[0] == robot_position[0] and r.position[1] + 1 == robot_position[1]:
                free_direction['up'] = False

        return free_direction

    def get_neighbor(self, robot, observation, direction):
        
        free_direction = self.check_own_robot(robot, observation)

        self.print("FREE DIRECTION", free_direction)

        if direction == 'left' and free_direction['left']:
            return self.map[robot.position[1]][robot.position[0] - 1]
        elif direction == 'left' and not free_direction['left']:
            return 'R'

        if direction == 'right' and free_direction['right']:
            return self.map[robot.position[1]][robot.position[0] + 1]
        elif direction == 'right' and not free_direction['right']:
            return 'R'
        
        if direction == 'up' and free_direction['up']:
            return self.map[robot.position[1] - 1][robot.position[0]]
        elif direction == 'up' and not free_direction['up']:
            return 'R'

        if direction == 'down' and free_direction['down']:
            return self.map[robot.position[1] + 1][robot.position[0]]
        elif direction == 'down' and not free_direction['down']:
            return 'R'

    def scan(self, robot_id, observation, action):
        robot = observation.robot(robot_id)

        # Check whether detection switch is ON or OFF. Switch it ON
        if not self.robot_state[robot_id]['detection_enabled']:
            action.detect(robot_id)
            self.robot_state[robot_id]['detection_enabled'] = True
            return
        
        self.robot_state[robot_id]['detection_enabled'] = False
        obj_right = observation.radar(robot_id, 'right')
        obj_left = observation.radar(robot_id, 'left')
        obj_down = observation.radar(robot_id, 'down')
        obj_up = observation.radar(robot_id, 'up')
        
        right_x = left_x = top_y = bottom_y = None

        right_x = robot.position[0] + obj_right.distance
        left_x = robot.position[0] - obj_left.distance
        top_y = robot.position[1] - obj_up.distance
        bottom_y = robot.position[1] + obj_down.distance

        if obj_right.object is not None and right_x is not None and obj_right.object:
            self.map[robot.position[1]][right_x] = self.object_map[obj_right.object]
            for i in range(robot.position[0], right_x):
                self.map[robot.position[1]][i] = 'O'

        if obj_left.object is not None and left_x is not None and obj_left.object:
            self.map[robot.position[1]][left_x] = self.object_map[obj_left.object]
            for i in range(robot.position[0], left_x, -1):
                self.map[robot.position[1]][i] = 'O'

        if obj_up.object is not None and top_y is not None and obj_up.object:
            self.map[top_y][robot.position[0]] = self.object_map[obj_up.object]
            for i in range(robot.position[1], top_y, -1):
                self.map[i][robot.position[0]] = 'O'

        if obj_down is not None and bottom_y is not None and obj_down.object:
            self.map[bottom_y][robot.position[0]] = self.object_map[obj_down.object]
            for i in range(robot.position[1], bottom_y):
                self.map[i][robot.position[0]] = 'O'

        self.print(self.map)

    def print_map(self):
        self.print("PRINTING MAP")
        for i in range(self.shape[0]):
            self.print(self.map[i])

    def get_path(self, robot_id, start, end):
        a = copy.deepcopy(self.map)
        for i in range(len(a)):
            for j in range(len(a[i])):
                if a[i][j] == 'O' or a[i][j] == 'H':
                    a[i][j] = 0
                else:
                    a[i][j] = 1

        m = []
        for i in range(len(a)):
            m.append([])
            for j in range(len(a[i])):
                m[-1].append(0)
        j, i = start
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

        # Get Moves
        moves = []
        for i in range(len(path)-1):
            if path[i+1][0] > path[i][0]: # down
                moves.append('down')
            elif path[i+1][0] < path[i][0]: # up
                moves.append('up')
            elif path[i+1][1] > path[i][1]: # right
                moves.append('right')
            elif path[i+1][1] < path[i][1]: # left
                moves.append('left')

        self.robot_state[robot_id]['optimal_path_set'] = True
        self.robot_state[robot_id]['optimal_path_to_home'] = moves

        
# Run strategy
if __name__ == "__main__":
    strategy = Simple()
    strategy.run()
