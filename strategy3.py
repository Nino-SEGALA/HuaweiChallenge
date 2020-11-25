# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Romain Deffayet
    Karl Gäfvert
'''

import numpy as np
from robot_explorers import Strategy
import sys


class Strat3(Strategy):
    '''
    Implementation of an simple strategy based on random movement
    (does not follow optimal path to coin even when in direct veiw)

    Attributes:
        directions (list): List of possible directions for all actions
        path (dict): Contains the trace of how each robot moved
    '''

    def __init__(self):
        super().__init__()

    def init(self):
        '''Called once before the first observation has been received.'''

        self.print('Init')
        self.print('num_robots: {}'.format(self.num_robots))
        self.print('shape: {}'.format(self.shape))
        self.print('max_energy: {}'.format(self.max_energy))
        self.print('coin_box (x,y): {}'.format(self.coin_box))

        # For representing directions with indices
        self.directions = ['up', 'right', 'down', 'left']
        # Values of the different object in the board_map
        self.map_values = {"home_base": 'H', "opponent_home_base": 'O', "wall": 'W', "coin": 'C', "fake_coin": 'F',
                           "robot": 'R', "free_square": '-', "unidentified": 'U', "unknown": 'X'}  # other value?
        # Depth of simulation for the best move
        self.depth = 3
        # Values for the best move todo: find the best hyperparameters
        self.hyperparameters = {"alpha": 1, "beta": 100, "gamma": -30, "delta": 1.2, "epsilon": -100, "zeta": -30}
        # Initialization of the known map
        self.board_map = None  # Will be initialize during the step 1
        self.distance_map = None  # Will be initialize during the step 1
        # Count at which step we are
        self.current_step = 0
        # Position of our home_base
        #self.home_base_position = (1, 1)  # Will be adapted during the step 1
        # Position of our robots
        self.robots_position = [[] for r in range(self.num_robots)]
        # Position where our robots should be on the next step
        self.robots_position_next_move = []
        # Position of our home_base
        self.home_base_positions = [(1, 1)]  # Will be adapted during the step 1
        # If the robot has a specific path to follow
        self.path = [[] for r in range(self.num_robots)]
        # If the robot reserve a coin for him (and will search it) | we stock the position of the coin
        self.robot_coin = [(-1, -1) for r in range(self.num_robots)]
        # robot_id of the robots which will place fake_coins near the opponent home_base
        self.robot_fake_coin = [0]  # the first robot will places fake_coins
        # we store the last positions of our robots to don't stay stuck while exploring
        self.explore_position = [[] for r in range(self.num_robots)]

    def step(self, observation):
        ''' Called every time an observation has been received

            Args:
                observations (strategy.Observation):
                    Container with all robots' observations

            Returns:
                actions (strategy.Action):
                    Container with all robots actions
        '''

        #self.print('score: {}'.format(observation.score))
        #self.print('opponent score: {}'.format(observation.opponent_score))
        #self.print('added_coins: {}'.format(observation.added_coins))

        # increase the step number

        self.current_step += 1
        # if we are at step 1 we identified our home_base
        if self.current_step == 1:
            self.positionHomeBase(observation)  # We play up or bottom ?
            self.board_map, self.distance_map = self.initMap()  # Initialization of the board_map and distance_map
            for robot_id in range(self.num_robots):  # Initialization of the position of the robots
                self.initRobotPosition(observation, robot_id)

        # Initialize empty action
        action = self.action()

        self.print("knn", self.knowMap())

        # Loop over robots / Actualization of the board_map and distance_map
        for robot_id in range(self.num_robots):
            # Get robot specific observation
            robot = observation.robot(robot_id)
            #self.print('Robot {}:'.format(robot_id))
            for direction in self.directions:
                radar = observation.radar(robot_id, direction)  # get radar
                self.actualizeMap(robot, direction, radar)  # we actualize the board_map and distance_map
            self.actualizeRobotPosition(observation, robot_id)  # we actualize the position of our robot on the map

        #self.print(self.board_map[0:5])

        # Loop over robots / Choose action
        for robot_id in range(self.num_robots):
            # Get robot specific observation
            robot = observation.robot(robot_id)
            position = tuple(robot.position)

            ### Robot is penalized, do nothing
            if robot.penalty > 0:
                #self.print('penalty: {}'.format(robot.penalty))
                continue

            ### In home base and low on energy
            if robot.home_base and robot.energy < (self.max_energy / 3):
                #self.print('recharge')
                action.recharge(robot_id)  # recharge
                continue

            ### Choice of the best action
            #move = self.bestMove(observation, robot)  # we get the best move
            move, place_fake_coin = self.nextMove(observation, robot_id)  # we get the next move
            if move is not None:
                action.move(robot_id, move)  # set move action
                # actualize robot position
                next_position = tuple(np.array(self.numpyPosition(position))
                                  + np.array(self.numpyPosition(self.dir2coord(move))))
                if self.detectionNextMove(position, move):  # if there is something to detect after the move
                    action.detect(robot_id)
                self.actualizeRobotAfterMove(robot_id, self.numpyPosition(position), next_position)
            if place_fake_coin:  # if we have a direction to place a fake_coin
                action.fake_coin(robot_id, place_fake_coin)
                self.actualizeFakeCoin(position, place_fake_coin)  # we store the place were we put fake_coins
                # todo solve problems
                # todo fake_coins placed near home_base
                # todo robots collecting fake_coins

        #self.print(f"step {self.current_step} : board_map")
        #self.print(self.board_map)

        return action

    # Calculate the position of our home_base
    def positionHomeBase(self, observation):
        x, y = self.shape
        robot = observation.robot(0)  # we take the first robot
        robot_x, robot_y = self.numpyPosition(tuple(robot.position))
        home_x, home_y = (1, 1)  # upper position

        # if the robot is at the top of the board, we have the upper home_base
        if robot_x < x - robot_x:
            self.home_base_positions[0] = (home_x, home_y)
        # if the robot is at the bottom of the board, we have the lower home_base
        else:
            self.home_base_positions[0] = self.symmetric((home_x, home_y))

    # Creates the known-map of the board
    def initMap(self):
        board_map = np.full(self.shape, self.map_values["unknown"])  # what is unknown is set to infinity
        distance_map = np.full(self.shape, np.inf)  # what is unknown is set to infinity

        ### side walls
        board_map[0:self.shape[0], 0] = self.map_values["wall"]  # left walls
        board_map[0:self.shape[0], self.shape[1] - 1] = self.map_values["wall"]  # right walls
        board_map[0, 0:self.shape[1]] = self.map_values["wall"]  # up walls
        board_map[self.shape[0] - 1, 0:self.shape[1] - 1] = self.map_values["wall"]  # bottom walls

        ### the home-bases
        # if we play at the top
        if self.home_base_positions[0] == (1, 1):
            home_base1 = "home_base"
            home_base2 = "opponent_home_base"
        # if we play at the bottom
        else:
            home_base1 = "opponent_home_base"
            home_base2 = "home_base"
        sym_x, sym_y = self.symmetric((1, 1))
        board_map[1][1] = self.map_values[home_base1]  # the upper home_base
        board_map[sym_x][sym_y] = self.map_values[home_base2]  # the bottom home_base

        distance_map[1][1] = 0  # distance to our home_base

        return board_map, distance_map

    # Initialize the position of the robots
    def initRobotPosition(self, observation, robot_id):
        robot = observation.robot(robot_id)
        x, y = self.numpyPosition(robot.position)
        self.robots_position[robot_id] = [x, y]  # we store the position of the robot (old one)
        self.board_map[x][y] = self.map_values["robot"]  # we store the current position on the board_map

    # Return the symmetric of position
    def symmetric(self, position):
        return self.shape[0] - position[0] - 1, position[1]

    # Return the symmetric object
    def symmetricObject(self, obj):
        #self.map_values = {"home_base": 'H', "opponent_home_base": 'O', "wall": 'W', "coin": 'C', "fake_coin": 'F',
        #                   "robot": 'R', "free_square": 0, "unidentified": 'U', "unknown": 'X'}  # other value?

        symmetric = [["home_base", "opponent_home_base"], ["opponent_home_base", "home_base"], ["wall", "wall"],
                     ["coin", "free_square"], ["fake_coin", "free_square"], ["robot", "free_square"],
                     ["free_square", "free_square"], ["unidentified", "unidentified"], ["unknown", "unknown"]]
        for (obj1, obj2) in symmetric:
            if obj == obj1:
                return obj2
        self.print("ERROR : symmetricObject, given object wasn't found")
        return None

    # Actualization of the board_map after the use of the radar
    def actualizeMap(self, robot, direction, radar):
        robot_x, robot_y = self.numpyPosition(tuple(robot.position))  # we get the position of the robot
        dir_x, dir_y = self.numpyPosition(self.dir2coord(direction))  # we get the direction (as a tuple)

        # we actualize the board_map and the distance_map with the value of the free_squares
        for i in range(radar.distance):
            x = robot_x + i * dir_x
            y = robot_y + i * dir_y
            sym_x, y = self.symmetric((x, y))  # the symmetric position
            bmxy = self.board_map[x][y]
            dmxy = self.distance_map[x][y]
            # if we already know there is a free_square, it stores the distance to the home_base
            # so we don't want to overwrite it
            try:
                v = int(dmxy)  # works only if dmxy is a free_square / our home_base, not if it is equal to np.inf
            except:
                if bmxy not in [self.map_values["home_base"], self.map_values["opponent_home_base"],
                                self.map_values["wall"]]:  # if it's not an immutable object
                    obj = "free_square"
                    sym_obj = self.symmetricObject(obj)  # the symmetrical corresponding object
                    # board_map
                    self.board_map[x][y] = self.map_values[obj]
                    self.board_map[sym_x][y] = self.map_values[sym_obj]
                    # distance_map
                    self.distance_map[x][y] = -1  # distance to home_base isn't evaluated
                    self.distance_map[sym_x][y] = -1  # distance to home_base isn't evaluated

        # position of the detected object
        x = robot_x + radar.distance * dir_x
        y = robot_y + radar.distance * dir_y
        sym_x, sym_y = self.symmetric((x, y))  # the symmetric position
        # we store the nature of the object in the map (we don't replace immutable items)
        if self.board_map[x][y] not in (self.map_values["home_base"],
                                        self.map_values["opponent_home_base"],
                                        self.map_values["wall"]):
            obj = radar.object
            if obj is None:  # if we didn't detect, the object is unidentified
                obj = "unidentified"
                sym_obj = self.symmetricObject(obj)  # the symmetrical corresponding object
                # don't replace other robots/coins with unidentified
                if self.board_map[x][y] not in (self.map_values["robot"],
                        self.map_values["coin"], self.map_values["fake_coin"]):
                    self.board_map[x][y] = self.map_values[obj]
                    self.board_map[sym_x][sym_y] = self.map_values[sym_obj]

            else:
                if obj == "robot":  # we don't want to actualize our map with the position of the robots by detection
                    obj = "free_square"
                sym_obj = self.symmetricObject(obj)  # the symmetrical corresponding object
                self.board_map[x][y] = self.map_values[obj]
                self.board_map[sym_x][sym_y] = self.map_values[sym_obj]

            # we detect something corresponding to a new free_square
            if (obj == "free_square" or obj == "coin") and self.distance_map[x][y] == np.inf:  # robot -> free_square
                self.distance_map[x][y] = -1  # distance to home_base isn't evaluated
                self.distance_map[sym_x][sym_y] = -1  # distance to home_base isn't evaluated

            # during the first step we can identify the other home_base boxes
            if self.current_step == 1:
                self.otherHomeBase((x, y), direction)

        # we update the distance_map with the distance to our home_base (from the new discovered free_squares)
        self.distanceMap()

    # Actualize the position of the robot on board_map
    def actualizeRobotPosition(self, observation, robot_id):
        robot = observation.robot(robot_id)
        new_position = self.numpyPosition(robot.position)
        position = self.robots_position[robot_id]  # the previous position of the robot
        self.actualizeRobotAfterMove(robot_id, position, new_position)

    # after chosen move, we actualize the map with the future position of the robot
    def actualizeRobotAfterMove(self, robot_id, position, next_position):
        #self.print("actualizeRobotAfterMove :", robot_id, position, next_position)
        old_x, old_y = position
        x, y = next_position
        # board_map
        if (old_x, old_y) in self.home_base_positions:  # we were in our home_base
            self.board_map[old_x][old_y] = self.map_values["home_base"]
        else:  # we were on a free_square
            self.board_map[old_x][old_y] = self.map_values["free_square"]
        self.board_map[x][y] = self.map_values["robot"]
        # robots_position
        self.robots_position[robot_id] = [x, y]  # the new position of the robot is stored

    # If the given position corresponds to another home_base boxes we add it
    # and calculate other home_base boxes (some of them)
    def otherHomeBase(self, position, direction):
        x, y = position
        if 0 < x < self.shape[0] and 0 < y < self.shape[1]:
            # if we play at the top
            if self.home_base_positions[0] == (1, 1):
                if direction in ("left", "up"):
                    self.detectedHomeBase(position)
            # if we play at the bottom
            else:
                if direction in ("left", "down"):
                    self.detectedHomeBase(position)

    # we replace the value of the position with the home_base value and we replace the opponent_home_base with its value
    # we call back detectedHomeBase on the other box we know which are home_base too
    def detectedHomeBase(self, position):
        x, y = position  # is already a numpyPosition
        sym_x, sym_y = self.symmetric(position)
        self.board_map[x][y] = self.map_values["home_base"]
        self.board_map[sym_x][sym_y] = self.map_values["opponent_home_base"]
        self.distance_map[x][y] = 0  # distance to our home_base
        if (x, y) not in self.home_base_positions:  # we add the position of our new detected home_base
            self.home_base_positions.append((x, y))

        ### we look at the other positions (can be improved !!!)
        (new_x, new_y) = tuple(np.array(position) + np.array(self.numpyPosition(self.dir2coord("left"))))  # new position
        if 0 < new_x < self.shape[0] and 0 < new_y < self.shape[1]:
            self.detectedHomeBase((new_x, new_y))
        # we play at the top
        if self.home_base_positions[0] == (1, 1):  # todo other positions
            (new_x, new_y) = tuple(np.array(position)
                                   + np.array(self.numpyPosition(self.dir2coord("up"))))  # new position
            if 0 < new_x < self.shape[0]-1 and 0 < new_y < self.shape[1]-1:
                self.detectedHomeBase((new_x, new_y))
        # we play at the bottom
        else:
            (new_x, new_y) = tuple(np.array(position)
                                   + np.array(self.numpyPosition(self.dir2coord("down"))))  # new position
            if 0 < new_x < self.shape[0]-1 and 0 < new_y < self.shape[1]-1:
                self.detectedHomeBase((new_x, new_y))

    # Calculate the distance to the home-base of every point of the distance_map
    def distanceMap(self):
        (n, p) = self.shape
        for x in range(1, n - 1):
            for y in range(1, p - 1):
                self.minValue((x, y))

    # return the minimum value of the neighbors + 1, and call minValue on the neighbors which value is > minValue+1
    def minValue(self, position):
        (x, y) = position  # already numpy position
        # if the position doesn't correspond to a free_square
        if self.distance_map[x][y] != -1:
            return None
        minValue = np.inf
        # neighbors
        v1 = self.distance_map[x - 1][y]  # up
        v2 = self.distance_map[x][y - 1]  # left
        v3 = self.distance_map[x][y + 1]  # right
        v4 = self.distance_map[x + 1][y]  # bottom

        # we look at the neighbors
        for v in [v1, v2, v3, v4]:
            if v >= 0 and v < minValue - 1:
                minValue = v + 1  # it takes 1 more step to achieve this position from the home_base

        # instantiation of the position's distance if it has change
        if minValue < np.inf:
            self.distance_map[x][y] = minValue
            # we adjust the value of the distance of the neighbors in the case it has to be
            for (v, pos) in [(v1, (x - 1, y)), (v2, (x, y - 1)), (v3, (x, y + 1)), (v4, (x + 1, y + 1))]:
                # if a neighbor has a wrong distance_value to our home_base, we actualize it
                if v == -1 or (v < np.inf and v > minValue + 1):
                    self.minValue(pos)

    # Convert a direction into the corresponding tuple (in games-axis !!)
    def dir2coord(self, direction):
        ''' Convert direction to coordinates

            Args:
                pos (tuple|list|np.array):
                    Container with x,y coordinates of start position
                direction (str):
                    Direction of action

            Returns:
                goal (tuple): Container with x,y coordinate of goal position

            Raises:
                ValueError: Not a valid direction
        '''
        switch = {
            'up': np.array([0, -1]),
            'down': np.array([0, 1]),
            'left': np.array([-1, 0]),
            'right': np.array([1, 0]),
            'stay': np.array([0, 0])
        }
        step = switch.get(direction, None)
        if step is None:
            self.print("ERROR : dir2coord : ", direction)
            raise ValueError("Not a valid direction ({})".format(direction))
        return tuple(step)

    # Convert a game-position, into a numpy_position
    def numpyPosition(self, position):
        x, y = position
        return y, x

    # Find the best move for one robot
    def bestMove(self, observation, robot):
        position = self.numpyPosition(tuple(robot.position))
        bestMove, value = self.bestMoveRec(observation, position, robot.energy, robot.has_item, self.depth)
        if bestMove == "":  # if no move lead to a finite value in bestMoveRec
            # choose random direction to move
            self.print("ERROR : random move chosen")
            bestMove = np.random.choice(list(self.directions))
        return bestMove

    # Recursive function to find the best move
    def bestMoveRec(self, observation, position, energy, have_coin, depth):
        # Leaf / we evaluate the board
        if depth == 0:
            x, y = position  # already numpyPosition
            distance_home_base = self.distanceHomeBase(position)
            distance_coin, (coin_x, coin_y), path = self.distanceCoin(position)
            distance_coin_home_base = self.distanceCoinHomeBase((coin_x, coin_y))

            # if there is no coin
            delta = -1
            if distance_coin != None:
                delta = 1 / distance_coin * (distance_coin_home_base < energy - distance_coin)

            # the heuristic function
            value = self.hyperparameters["beta"] * observation.score \
                    + self.hyperparameters["gamma"] * observation.opponent_score \
                    + self.hyperparameters["delta"] * (1 - have_coin) * delta \
                    + self.hyperparameters["epsilon"] * have_coin * distance_home_base \
                    + self.hyperparameters["zeta"] * (distance_home_base > energy)
            # self.hyperparameters["alpha"] * self.knowMap()  # problem: it's the same for every position
            # self.hyperparameters = {"alpha": 1, "beta": 100, "gamma": -30, "delta": 1.2, "epsilon": 1.5, "zeta": -30}
            return "", value
        else:
            bestMove = ""
            maxValue = -np.inf
            for move in self.directions:
                (x, y) = tuple(np.array(position) + np.array(self.numpyPosition(self.dir2coord(move))))  # new position
                bmxy = self.board_map[x][y]
                if bmxy == self.map_values["coin"]:  # if we collect a coin
                    have_coin = True
                # we only go to known free_squares
                try:
                    v = int(bmxy)
                    if [x, y] in self.robots_position:  # we never go where one of our robot stand
                        return "", -np.inf
                except:  # if it's not an int
                    if not (bmxy == self.map_values["home_base"] or bmxy == self.map_values["coin"]):
                        return "", -np.inf
                # recursive call
                bstMv, value = self.bestMoveRec(observation, (x, y), energy - 1 - have_coin, have_coin, depth - 1)
                if value > maxValue:
                    bestMove = move
                    maxValue = value
        return bestMove, value

    # Return the distance to our home_base
    def distanceHomeBase(self, position):
        x, y = position  # already numpyPosition
        dist = self.distance_map[x][y]
        if dist == np.inf:
            self.print("ERROR : distanceHomeBase, given position doesn't correspond to a free_square")
        elif dist == -1:
            self.print("ERROR : distanceHomeBase, the distance wasn't evaluated")
        return dist

    # Return the proportion of the distance_map that we know
    def knowMap(self):
        (n, p) = self.shape
        prop = 0
        for x in range(1, n - 1):  # we don't look at the borders
            for y in range(1, p - 1):  # we don't look at the borders
                # what with the "unidentified" objects ?
                if self.distance_map[x][y] != np.inf:  # if we know what is in the position (x, y)
                    prop += 1
        return prop / ((n - 2) * (p - 2))  # the dimensions of the distance_map without the borders

    # Calculate the shortest distance to a coin
    def distanceCoin(self, position):
        robot_x, robot_y = position  # already numpyPosition
        distance = -1  # distance to the nearest coin
        pos = (-1, -1)  # position of the nearest coin
        dist_map = np.full(self.shape, np.inf)
        dist_map[robot_x][robot_y] = 0  # distance from our robot to itself
        squares = []  # list of the squares/boxes that will be evaluated
        path = []

        # look at the neighbors and adjust their distance
        """for (x, y) in self.neighbors(position):
            #if self.board_map[x][y] != self.map_values["robot"]:  # we don't go over a robot
            if self.board_map[x][y] == self.map_values["free_square"] \
                    or self.board_map[x][y] != self.map_values["home_base"]:
                if self.distance_map[x][y] != np.inf:  # if it's a free_square
                    squares.append((x, y))
                    dist_map[x][y] = 1"""
        squares.append(position)

        # searching for the nearest coin
        while distance == -1:
            if len(squares) == 0:
                break
            x, y = squares.pop(0)  # BFS
            dist = dist_map[x][y]
            # if there is a coin
            if self.board_map[x][y] == self.map_values["coin"]:
                if (x, y) not in self.robot_coin:  # if the coin isn't yet reserved by another robot
                    distance = dist
                    pos = (x, y)
                    break
            nghb = self.neighbors((x, y))
            for (i, j) in nghb:
                # if we don't have already visit it
                #if self.board_map[i][j] != self.map_values["robot"]:  # we don't go over a robot
                if self.board_map[i][j] == self.map_values["free_square"] \
                        or self.board_map[i][j] == self.map_values["home_base"]\
                        or self.board_map[i][j] == self.map_values["coin"]:
                    if dist_map[i][j] == np.inf:
                        if self.distance_map[i][j] != np.inf:  # if it's a free_square
                            squares.append((i, j))
                            dist_map[i][j] = dist + 1

        # the path from the coin to the robot
        if pos != (-1, -1):  # if a coin were found
            path.append(pos)  # we add the coin's position
            while path[-1] != position:  # the path is incomplete (don't reach the robot yet)
                x, y = path[-1]
                nghb = self.neighbors(path[-1])  # neighbors of the last square
                for (i, j) in nghb:
                    if dist_map[i][j] == dist_map[x][y]-1:  # we find the path to the robot
                        path.append((i, j))
                        break
            path.pop()  # we delete the position of the robot of the path
            path.reverse()  # we reverse it to have the path from the robot to the coin

            #next_x, next_y = path[0]
            #self.print("distanceCoin : ", position, pos, path, self.board_map[next_x][next_y])

        return distance, pos, path

    # Calculate the distance between the coin and our home_base
    def distanceCoinHomeBase(self, position):
        coin_x, coin_y = position  # already in numpyPosition
        distance = self.distance_map[coin_x][coin_y]
        return distance

    # Return the neighbors of a position
    def neighbors(self, position):
        x, y = position
        nghb = []
        if x > 1:
            nghb.append((x - 1, y))
        if x < self.shape[0] - 2:
            nghb.append((x + 1, y))
        if y > 1:
            nghb.append((x, y - 1))
        if y < self.shape[1] - 2:
            nghb.append((x, y + 1))
        return nghb

    # evaluate the next move of the robot
    def nextMove(self, observation, robot_id):
        # Get robot specific observation
        robot = observation.robot(robot_id)
        position = self.numpyPosition(tuple(robot.position))  # get the position of the robot
        go_back_home = False  # if not enough energy, we go back home

        # if the robot is an impostor trying to place fake_coins
        if robot_id in self.robot_fake_coin:
            # if the robot has no specific path, we look if we can assign one to it
            if self.path[robot_id] == []:
                dir = self.placingFakeCoin(position, robot.energy)  # we look if we should place a fake_coin
                if dir:
                    return None, dir  # we return the direction in which we want to place a fake_coin
                path = self.pathFakeCoin(position)  # we look if we can get closer to the home_base
                if path is not None:  # there is a way to the opponent_home_base
                    if path != []:
                        self.path[robot_id] = [path[0]]  # we put only the next move, to avoid skipped moves problem
                    # no path were found, we should have reach our goal
                    else:
                        dir = self.placingFakeCoin(position, robot.energy, place=True)  # we place a fake_coin
                        if dir:
                            return None, dir  # we return the direction in which we want to place a fake_coin

        # if the robot has no specific path, we look if we can assign one to it
        if self.path[robot_id] == []:
            # if the robot has no coin, we look if he can search a free coin (not already assigned to another robot)
            if not robot.has_item:
                dist, pos, path = self.distanceCoin(position)
                if dist > 0:  # if we found a free coin
                    x, y = pos
                    # enough energy to search the coin
                    # enough_energy_coin = robot.energy - dist - 2 * self.distance_map[x][y] > 5
                    # if enough_energy_coin:
                    self.path[robot_id] = [path[0]]  # we put only the next move, to avoid skipped moves problem
                    self.print("pathCoin: ", position, path, robot_id, self.path)
                    # else:
                    #     go_back_home = True  # not enough energy to search a coin: we go back home

            # the robot has a coin but doesn't have the path to go home -> we assign the path to go to the home_base
            if robot.has_item or go_back_home:
                path = self.pathHomeBase(position)
                if path != []:  # should be the case since the robot is on a valid position
                    if path == "stay":  # when we wait to have access to the home_base
                        move = None
                        return move, False  # we don't place a fake_coin
                    self.path[robot_id] = [path[0]]  # we put only the next move, to avoid the problem of skipped moves
                    self.print("pathHomeBase: ", position, path, robot_id, self.path)

        # if the robot should follow a specific path
        if self.path[robot_id] != []:
            next_position = self.path[robot_id].pop(0)  # get the next position and remove it from path
            move = self.coord2dir(position, next_position)  # the move to go to next_position
            return move, False  # we return the move corresponding to the path the robot has to follow

        # exploration : no coins to search or to bring back home
        move = self.exploration(robot_id, position)
        self.print("exploration :", move, ", robot : ", robot_id)
        return move, False  # we don't place a fake_coin

        #return None  # should not happen

    # gives the move to go to the goal
    def coord2dir(self, pos, goal):
        ''' Convert coordinates to direction

            Args:
                pos (tuple|list|np.array):
                    Container with x,y coordinates of start position
                goal (tuple|list|np.array):
                    Container with x,y coordinates of goal position

            Returns:
                actions (str): The direction to use in action

            Raises:
                ValueError: Starting position and goal are not adjacent
        '''
        # position inverted like always :/
        pos = pos[1], pos[0]
        goal = goal[1], goal[0]

        if pos == goal:
            return None
        elif pos[0] == goal[0]:
            if pos[1] > goal[1]:
                return "up"
            else:
                return "down"
        elif pos[1] == goal[1]:
            if pos[0] > goal[0]:
                return "left"
            else:
                return "right"
        else:
            self.print("ERROR coord2dir : ", pos, goal)
            raise ValueError("Starting position and goal are not adjacent")

    # the path to our home_base
    def pathHomeBase(self, position):
        robot_x, robot_y = position  # already numpyPosition
        distance = -1  # distance to the nearest home_base
        pos = (-1, -1)  # position of the nearest home_base
        dist_map = np.full(self.shape, np.inf)
        dist_map[robot_x][robot_y] = 0  # distance from our robot to itself
        squares = []  # list of the squares/boxes that will be evaluated
        path = []

        # look at the neighbors and adjust their distance
        """for (x, y) in self.neighbors(position):
            if self.board_map[x][y] == self.map_values["free_square"]\
                    or self.board_map[x][y] == self.map_values["home_base"]:  # if it's a free_square
                squares.append((x, y))
                dist_map[x][y] = 1"""

        squares.append(position)

        # searching for the nearest home_base
        while distance == -1:
            if len(squares) == 0:
                break
            x, y = squares.pop(0)  # BFS
            dist = dist_map[x][y]
            # if there is our home_base
            if self.board_map[x][y] == self.map_values["home_base"]:
                distance = dist
                pos = (x, y)
                break
            nghb = self.neighbors((x, y))
            for (i, j) in nghb:
                # if we don't have already visit it
                #if self.board_map[i][j] != self.map_values["robot"]:  # we don't go over a robot
                if self.board_map[i][j] == self.map_values["free_square"] \
                        or self.board_map[i][j] == self.map_values["home_base"]:  # if it's a free_square
                    if dist_map[i][j] == np.inf:  # we don't want to make a loop
                        squares.append((i, j))
                        dist_map[i][j] = dist + 1

        # when something (like robots) block the way back to the home_base
        if pos == (-1, -1):
            dist_robot = self.distance_map[robot_x][robot_y]  # the distance to our home_base
            distance = -1  # distance of the secured path
            length = min(5, dist_robot-2)  # the length of the secure path to our home_base
            dist_map = np.full(self.shape, np.inf)
            dist_map[robot_x][robot_y] = 0  # distance from our robot to itself
            squares = []  # list of the squares/boxes that will be evaluated
            path = []

            # we are waiting newt to an occupied home_base
            if dist_robot == 1:
                return "stay"

            # look at the neighbors and adjust their distance
            for (x, y) in self.neighbors(position):
                if self.board_map[x][y] == self.map_values["free_square"] \
                        or self.board_map[x][y] == self.map_values["home_base"]:  # if it's a free_square
                    squares.append((x, y))
                    dist_map[x][y] = 1

            # look at a way of the good length
            while distance == -1:
                if len(squares) == 0:
                    break
                x, y = squares.pop(0)  # BFS
                dist = dist_map[x][y]
                if self.distance_map[x][y] <= dist_robot - length:
                    distance = dist_robot - self.distance_map[x][y]
                    pos = (x, y)
                    break
                nghb = self.neighbors((x, y))
                for (i, j) in nghb:
                    # if we don't have already visit it
                    if self.board_map[i][j] == self.map_values["free_square"] \
                            or self.board_map[i][j] == self.map_values["home_base"]:  # if it's a free_square
                        if dist_map[i][j] == np.inf:  # we don't want to make a loop
                            squares.append((i, j))
                            dist_map[i][j] = dist + 1

        # the path from the robot to the home_base
        if pos != (-1, -1):  # if a home_base were found
            path.append(pos)  # we add the home_base's position
            while path[-1] != position:  # the path is incomplete (don't reach the robot yet)
                x, y = path[-1]
                nghb = self.neighbors(path[-1])  # neighbors of the last square
                for (i, j) in nghb:
                    if dist_map[i][j] == dist_map[x][y] - 1:  # we find the path to the robot
                        path.append((i, j))
                        break
            path.pop()  # we delete the position of the robot of the path
            path.reverse()  # we reverse it to have the path from the robot to the coin

        return path

    # exploration strategy
    def exploration(self, robot_id, position):
        self.print("EXPLORE", self.explore_position)
        number_past_moves = 5  # we try to not go into one of the last 5 positions of our exploring robot
        x, y = position  # already numpyPosition

        # we keep the 5 last positions (should already be the case)
        while len(self.explore_position[robot_id]) > 5:
            self.explore_position[robot_id].pop(0)

        # we define the direction priority
        if self.home_base_positions[0] == (1, 1):  # we play at the top
            directions = ["right", "down", "left", "up"]
            if robot_id in self.robot_fake_coin:  # if the robots try to find a path to the opponent home_base
                directions = ["down", "left", "right", "up"]
        else:  # we play at the bottom
            directions = ["right", "up", "left", "down"]
            if robot_id in self.robot_fake_coin:  # if the robots try to find a path to the opponent home_base
                directions = ["up", "left", "right", "down"]

        # we look at the possible moves
        move, importance = None, -1  # the chosen move, the importance of the move (avoid the last positions)
        move_x, move_y = None, None
        for d in directions:
            new_x, new_y = tuple(np.array(position) + np.array(self.numpyPosition(self.dir2coord(d))))
            if self.board_map[new_x][new_y] == self.map_values["free_square"]:
                # we assign the importance of the new position
                impt = number_past_moves + 2
                for i in range(len(self.explore_position[robot_id])):
                    if self.explore_position[robot_id][i] == (new_x, new_y):
                        impt -= i  #todo other way
                        break

                # if the new importance is the highest, we keep this move
                if impt > importance:
                    move = d
                    importance = impt
                    move_x = new_x
                    move_y = new_y

        # if we found a move
        if move is not None:
            self.explore_position[robot_id].append((move_x, move_y))
            return move

        self.print("ERROR : exploration, no valid move founded")
        # move = np.random.choice(list(self.directions))
        move = "stay"
        return move

    # if there is something to detect on the next move, we detect
    def detectionNextMove(self, position, move):
        x, y = self.numpyPosition(position)
        next_x, next_y = tuple(np.array((x, y)) + np.array(self.numpyPosition(self.dir2coord(move))))

        # if the chosen move is strange
        if self.board_map[next_x][next_y] != self.map_values["free_square"]\
                and self.board_map[next_x][next_y] != self.map_values["home_base"]\
                and self.board_map[next_x][next_y] != self.map_values["coin"]:
            return False
        # and self.board_map[next_x][next_y] != self.map_values["robot"]:  # for the "stay" move

        # what we want to detect
        detectObj = [self.map_values["unidentified"], self.map_values["unknown"]]

        directions = ["right", "down", "left", "up"]
        for d in directions:
            dist, obj = self.radarDirection((next_x, next_y), d)  # we look, what we could see at the next step
            if obj in detectObj:  # if we want to discover the nature of this object
                self.print("detect : True")
                return True
        return False

    # returns the distance and the nature of the nearest object in the given direction
    def radarDirection(self, position, direction):
        x, y = position  # position of the robot at the next step
        n, p = self.numpyPosition(self.dir2coord(direction))  # vector corresponding to the direction

        i, j = x+n, y+p  # the neighbor box
        dist = 1
        if i < 0 or i > self.shape[0] - 1 or j < 0 or j > self.shape[1] - 1:
            self.print("POSITION POSITION", (x, y), direction, (i, j))
            self.print(self.board_map)
        while self.board_map[i][j] == self.map_values["free_square"]:
            # we move one box further
            i += n
            j += p
            dist += 1  # we are at a distance 1 further
            if i < 0 or i > self.shape[0]-1 or j < 0 or j > self.shape[1]-1:
                self.print("POSITION POSITION", (x, y), direction, (i, j))
                self.print(self.board_map)
        return dist, self.board_map[i][j]

    # find a way to a place near the opponent home_base and place a fake_coin
    def pathFakeCoin(self, position):
        robot_x, robot_y = position

        hx, hy = 1, 1
        while self.board_map[hx][hy] == self.map_values["home_base"] \
            or self.board_map[hx][hy] == self.map_values["opponent_home_base"]:
            hx += 1
            hy += 1
        #to avoid problem with a robot staying on a home_base
        hx += 1
        hy += 1
        # we play at the top
        if self.home_base_positions[0] == (1, 1):
             hx, hy = self.symmetric((hx, hy))

        # we look if there is no path from our home base to the opponent_home_base
        if self.distance_map[hx][hy] == np.inf or self.distance_map[hx][hy] == 0:
            return None

        # if there is a path
        pos = (-1, -1)  # position of our goal
        dist_map = np.full(self.shape, np.inf)
        dist_map[robot_x][robot_y] = 0  # distance from our robot to itself
        squares = []  # list of the squares/boxes that will be evaluated
        path = []

        # look at the neighbors and adjust their distance
        """for (x, y) in self.neighbors(position):
            #if self.board_map[x][y] != self.map_values["robot"]:  # we don't go over a robot
            if self.board_map[x][y] == self.map_values["free_square"]\
                    or self.board_map[x][y] != self.map_values["home_base"]:
                if self.distance_map[x][y] != np.inf:  # if it's a free_square
                    squares.append((x, y))
                    dist_map[x][y] = 1"""

        squares.append(position)

        # searching for the path to the opponent_home_base
        while len(squares) > 0:
            x, y = squares.pop(0)  # BFS
            dist = dist_map[x][y]
            nghb = self.neighbors((x, y))
            for (i, j) in nghb:
                # if we don't have already visit it
                if dist_map[i][j] == np.inf:
                    """if self.board_map[i][j] != self.map_values["robot"]\
                            and self.board_map[i][j] != self.map_values["coin"]\
                            and self.board_map[i][j] != self.map_values["fake_coin"]:  # we go only over free_square"""
                    if self.board_map[i][j] == self.map_values["free_square"] \
                            or self.board_map[i][j] == self.map_values["home_base"]:
                        if self.distance_map[i][j] != np.inf:  # if it's a free_square
                            squares.append((i, j))
                            dist_map[i][j] = dist + 1

        # looking for our goal todo problem greedy 1 move
        dist = np.inf
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                if dist_map[i][j] < np.inf:  # accessible
                    if (hx + hy) - (i + j) < dist:
                        dist = (hx + hy) - (i + j)
                        pos = (i, j)

        # the path from the robot to our goal
        if pos != (-1, -1):  # if a path were found
            path.append(pos)  # we add the goal's position
            while path[-1] != position:  # the path is incomplete (don't reach the robot yet)
                x, y = path[-1]
                nghb = self.neighbors(path[-1])  # neighbors of the last square
                for (i, j) in nghb:
                    if dist_map[i][j] == dist_map[x][y] - 1:  # we find the path to the robot
                        path.append((i, j))
                        break
            path.pop()  # we delete the position of the robot of the path
            path.reverse()  # we reverse it to have the path from the robot to our goal
            if path != []:
                path.pop()  # we delete the last move (we will place a fake_coin there)

        return path

    # decide if we place a fake_coin
    def placingFakeCoin(self, position, energy, place=False):
        # if we are at our goal position and want to place a fake_coin
        if place:
            if energy >= 40:
                return self.placeFakeCoin(position)
        # if we can place 2 fake_coins or 1
        if 40-1 < energy < 40+5:  # 2*40-1 < energy < 2*40+5 or
            return self.placeFakeCoin(position)

        return None

    # place the fake_coin
    def placeFakeCoin(self, position):
        hx, hy = self.home_base_positions[0]
        if (hx, hy) == (1, 1):  # we play at the top
            directions = ["left", "down", "right", "up"]
        else:  # we play at the bottom
            directions = ["left", "up", "right", "down"]

        # we look in every direction and if it's possible to place a fake_coin we return the direction
        for d in directions:
            new_x, new_y = tuple(np.array(position) + np.array(self.numpyPosition(self.dir2coord(d))))  # neighbor
            if self.board_map[new_x][new_y] == self.map_values["free_square"]:
                return d

        return None

    # we store the place were we put fake_coins
    def actualizeFakeCoin(self, position, place_fake_coin):
        x, y = self.numpyPosition(position)  # we convert the position
        # the position of the fake_coin
        fc_x, fc_y = tuple(np.array((x, y)) + np.array(self.numpyPosition(self.dir2coord(place_fake_coin))))

        self.board_map[fc_x][fc_y] = self.map_values["fake_coin"]

# think about this problems after
# delete coin_position in self.robot_coin after picking it up (if a new coin appear there it's free)
# strange: board_map[x][y] = 'U' and distance_map[x][y] = 2.  (could be a coin which poped there)
# problem: detectionCoin gives path through other robot


# Run strategy
if __name__ == "__main__":
    strat3 = Strat3()
    strat3.run()
