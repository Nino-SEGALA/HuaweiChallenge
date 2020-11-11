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

        self.object_map = {
            'wall': 'W',
            'coin': 'C',
            'home_base': 'H',
            'robot': 'R'
        }
        self.map = []
        self.has_scouts = False

        self.scout_id = 0

        self.robot_state = {}

        for robot_id in range(self.num_robots):
            self.robot_state[robot_id] = {
                'detection_enabled': False,
                'priority_path' : [], # If robot has to go to a coin or come back to home base, we call get_path()
                'priority_path_index': None,
                'role': 'W',
                'going_for_coin': False,
                'traversed_path_from_home_base': []
            }

        if self.num_robots > 2:
            self.robot_state[2]['role'] = 'S'
            self.robot_state[3]['role'] = 'S'
            self.has_scouts = True

        for i in range(self.shape[0]):
            new_row = []
            for j in range(self.shape[1]):
                new_row.append('X')
            self.map.append(new_row)


    def step(self, observation):
        action = self.action()
        robot = observation.robot(self.scout_id)

        robot_state = self.robot_state[self.scout_id]

        self.print('DIRECTION', robot_state['traversed_path_from_home_base'])

        # Robot has coin, return to home base
        if robot.has_item:
            if len(robot_state['traversed_path_from_home_base']) > 0:
                direction_to_move = robot_state['traversed_path_from_home_base'][0]
                self.print("HOME BASE CALLING", direction_to_move)
                robot_state['traversed_path_from_home_base'].pop(0)
                action.move(self.scout_id, direction_to_move)
                return action

            # Last step to home
            if self.look_straight(robot, 'left') == 'H':
                action.move(self.scout_id, 'left')
                return action
            if self.look_straight(robot, 'right') == 'H':
                action.move(self.scout_id, 'right')
                return action
            if self.look_straight(robot, 'up') == 'H':
                action.move(self.scout_id, 'up')
                return action
            if self.look_straight(robot, 'down') == 'H':
                action.move(self.scout_id, 'down')
                return action

            # If home not found
            if self.look_straight(robot, 'left') == 'X':
                self.scan(self.scout_id, observation, action, 'left')
                self.print("SCANNING LEFT")
                return action
            
            if self.look_straight(robot, 'down') == 'X':
                robot_state['going_for_coin'] = False
                self.scan(self.scout_id, observation, action, 'down')
                self.print("SCANNING DOWN")
                return action

            if self.look_straight(robot, 'left') == 'X':
                robot_state['going_for_coin'] = False
                self.scan(self.scout_id, observation, action, 'left')
                self.print("SCANNING LEFT")
                return action

            if self.look_straight(robot, 'up') == 'X':
                robot_state['going_for_coin'] = False
                self.scan(self.scout_id, observation, action, 'up')
                self.print("SCANNING UP")
                return action


        # Spotted a coin, Move towards coin
        if self.look_straight(robot, 'right') == 'C':
            robot_state['going_for_coin'] = True
            self.move_right(self.scout_id, observation, action)
            robot_state['traversed_path_from_home_base'].insert(0, 'left')
            self.print("MOVING RIGHT")
            if self.get_neighbor(robot, 'right') == 'C':
                self.map[robot.position[1]][robot.position[0] + 1] = 'O'
            return action

        if self.look_straight(robot, 'left') == 'C':
            robot_state['going_for_coin'] = True
            self.move_left(self.scout_id, observation, action)
            robot_state['traversed_path_from_home_base'].insert(0, 'right')
            self.print("MOVING LEFT")
            if self.get_neighbor(robot, 'left') == 'C':
                self.map[robot.position[1]][robot.position[0] - 1] = 'O'
            return action

        if self.look_straight(robot, 'down') == 'C':
            robot_state['going_for_coin'] = True
            self.move_down(self.scout_id, observation, action)
            robot_state['traversed_path_from_home_base'].insert(0, 'up')
            self.print("MOVING DOWN")
            if self.get_neighbor(robot, 'down') == 'C':
                self.map[robot.position[1] + 1][robot.position[0]] = 'O'
            return action

        if self.look_straight(robot, 'up') == 'C':
            robot_state['going_for_coin'] = True
            self.move_up(self.scout_id, observation, action)
            robot_state['traversed_path_from_home_base'].insert(0, 'down')
            self.print("MOVING UP")
            if self.get_neighbor(robot, 'up') == 'C':
                self.map[robot.position[1] - 1][robot.position[0]] = 'O'
            return action
        

        # Scan
        if self.look_straight(robot, 'right') == 'X':
            robot_state['going_for_coin'] = False
            self.scan(self.scout_id, observation, action, 'right')
            self.print("SCANNING RIGHT")
            return action

        if self.look_straight(robot, 'down') == 'X':
            robot_state['going_for_coin'] = False
            self.scan(self.scout_id, observation, action, 'down')
            self.print("SCANNING DOWN")
            return action

        if self.look_straight(robot, 'left') == 'X':
            robot_state['going_for_coin'] = False
            self.scan(self.scout_id, observation, action, 'left')
            self.print("SCANNING LEFT")
            return action

        if self.look_straight(robot, 'up') == 'X':
            robot_state['going_for_coin'] = False
            self.scan(self.scout_id, observation, action, 'up')
            self.print("SCANNING UP")
            return action


        # Nothing to do, if neighbor is empty, take 1 move towards it
        if self.get_neighbor(robot, 'right') == 'O':
            action.move(self.scout_id, 'right')
            robot_state['traversed_path_from_home_base'].insert(0, 'left')
            self.print("MOVING RIGHT")
            return action
        
        if self.get_neighbor(robot, 'down') == 'O':
            action.move(self.scout_id, 'down')
            robot_state['traversed_path_from_home_base'].insert(0, 'up')
            self.print("MOVING DOWN")
            return action

        if self.get_neighbor(robot, 'left') == 'O':
            action.move(self.scout_id, 'left')
            robot_state['traversed_path_from_home_base'].insert(0, 'right')
            self.print("MOVING LEFT")
            return action

        if self.get_neighbor(robot, 'up') == 'O':
            action.move(self.scout_id, 'up')
            robot_state['traversed_path_from_home_base'].insert(0, 'down')
            self.print("MOVING UP")
            return action


        return action

    def move_down(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        if robot.position[1]+1 < self.shape[1]:
            if self.map[robot.position[1]+1][robot.position[0]] not in ('W', 'R', 'X'):
                action.move(robot_id, 'down')
                return True

        return False
    
    def move_left(self, robot_id, observation, action):
        robot = observation.robot(robot_id)

        if self.map[robot.position[1]][robot.position[0]-1] not in ('W', 'R', 'X'):
            action.move(robot_id, 'left')
            return True

        return False

    def move_right(self, robot_id, observation, action):
        robot = observation.robot(robot_id)

        if self.map[robot.position[1]][robot.position[0]+1] not in ('W', 'R', 'X'):
            action.move(robot_id, 'right')
            return True

        return False
    
    def move_up(self, robot_id, observation, action):
        robot = observation.robot(robot_id)
        if self.map[robot.position[1]-1][robot.position[0]]  not in ('W', 'R', 'X'):
            action.move(robot_id, 'up')
            return True
        return False


    def look_straight(self, robot, direction):
        robot_x = robot.position[0]
        robot_y = robot.position[1]
        if direction == 'right':
            while self.map[robot_y][robot_x + 1] == 'O':
                robot_x += 1
            return self.map[robot_y][robot_x + 1]

        if direction == 'left':
            while self.map[robot_y][robot_x - 1] == 'O':
                robot_x -= 1
            return self.map[robot_y][robot_x - 1]

        if direction == 'up':
            while self.map[robot_y - 1][robot_x] == 'O':
                robot_y -= 1
            return self.map[robot_y - 1][robot_x]
        
        if direction == 'down':
            while self.map[robot_y + 1][robot_x] == 'O':
                robot_y += 1
            return self.map[robot_y + 1][robot_x]

    def get_neighbor(self, robot, direction):
        if direction == 'left':
            return self.map[robot.position[1]][robot.position[0] - 1]
        if direction == 'right':
            return self.map[robot.position[1]][robot.position[0] + 1]
        if direction == 'up':
            return self.map[robot.position[1] - 1][robot.position[0]]
        if direction == 'down':
            return self.map[robot.position[1] + 1][robot.position[0]]

    def scan(self, robot_id, observation, action, direction):
        robot = observation.robot(robot_id)
        if not self.robot_state[robot_id]['detection_enabled']:
            action.detect(robot_id)
            self.robot_state[robot_id]['detection_enabled'] = True
            return
        
        self.robot_state[robot_id]['detection_enabled'] = False
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

        if obj.object:
            if right_x is not None:
                self.map[robot.position[1]][right_x] = self.object_map[obj.object]
                for i in range(robot.position[0], right_x):
                    self.map[robot.position[1]][i] = 'O'
            if left_x is not None:
                self.map[robot.position[1]][left_x] = self.object_map[obj.object]
                for i in range(robot.position[0], left_x, -1):
                    self.map[robot.position[1]][i] = 'O'
            if top_y is not None:
                self.map[top_y][robot.position[0]] = self.object_map[obj.object]
                for i in range(robot.position[1], top_y, -1):
                    self.map[i][robot.position[0]] = 'O'
            if bottom_y is not None:
                self.map[bottom_y][robot.position[0]] = self.object_map[obj.object]
                for i in range(robot.position[1], bottom_y):
                    self.map[i][robot.position[0]] = 'O'

    def print_map(self):
        for i in range(self.shape[0]):
            self.print(self.map[i])

    def get_path(self, start, end):
        a = self.map
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

        self.print(m)
        return
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
        
        self.print("HERE")


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
