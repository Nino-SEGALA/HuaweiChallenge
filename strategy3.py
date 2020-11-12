# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Romain Deffayet
    Karl Gäfvert
'''

import numpy as np
from robot_explorers import Strategy


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
                           "robot": 'R', "free_square": 0, "unidentified": 'U', "unknown": 'X'}  # other value?
        # Depth of simulation for the best move
        self.depth = 3
        # Values for the best move todo: find the best hyperparameters
        self.hyperparameters = {"alpha": 1, "beta": 100, "gamma": -30, "delta": 1.2, "epsilon": 100, "zeta": -30}
        # Initialization of the known map
        self.board_map = None  # Will be initialize during the step 1
        # Count at which step we are
        self.current_step = 0
        # Position of our home_base
        self.home_base_position = (1, 1)  # Will be adapted during the step 1
        self.print("end init")

    def step(self, observation):
        ''' Called every time an observation has been received

            Args:
                observations (strategy.Observation):
                    Container with all robots' observations

            Returns:
                actions (strategy.Action):
                    Container with all robots actions
        '''

        self.print('score: {}'.format(observation.score))
        self.print('opponent score: {}'.format(observation.opponent_score))
        self.print('added_coins: {}'.format(observation.added_coins))

        # increase the step number
        self.current_step += 1
        # if we are at step 1 we identified our home_base
        if self.current_step == 1:
            self.positionHomeBase(observation)  # We play up or bottom ?
            self.board_map = self.initMap()  # Initialization of the board_map

        # Initialize empty action
        action = self.action()

        # Loop over robots / Actualization of the board_map
        for robot_id in range(self.num_robots):
            # Get robot specific observation
            robot = observation.robot(robot_id)
            self.print('Robot {}:'.format(robot_id))
            for direction in self.directions:
                radar = observation.radar(robot_id, direction)  # get radar
                self.actualizeMap(robot, direction, radar)  # we actualize the board_map

        # Loop over robots / Choose action
        for robot_id in range(self.num_robots):
            # Get robot specific observation
            robot = observation.robot(robot_id)
            # position = tuple(robot.position)

            ### Robot is penalized, do nothing
            if robot.penalty > 0:
                self.print('penalty: {}'.format(robot.penalty))
                continue

            ### In home base and low on eneregy
            if robot.home_base and robot.energy < (self.max_energy / 3):
                self.print('recharge')
                action.recharge(robot_id)  # recharge

            ### Choice of the best action
            self.print("bestMove evaluation")
            # move = self.bestMove(observation, robot)  # we get the best move
            move = "right"
            self.print(move)
            action.move(robot_id, move)  # set move action
            action.detect(robot_id)  # we detect what we see

        self.print(f"step {self.current_step} : board_map")
        self.print(self.board_map)

        return action

    # Calculate the position of our home_base
    def positionHomeBase(self, observation):
        x, y = self.shape
        robot = observation.robot(0)  # we take the first robot
        robot_x, robot_y = tuple(robot.position)
        home_x, home_y = (1, 1)  # upper position

        # if the robot is at the top of the board, we have the upper home_base
        if robot_x < x - robot_x:
            self.home_base_position = home_x, home_y
        # if the robot is at the bottom of the board, we have the lower home_base
        else:
            self.home_base_position = self.symmetric((home_x, home_y))

    # Creates the known-map of the board
    def initMap(self):
        board_map = np.full(self.shape, self.map_values["unknown"])  # what is unknown is set to infinity

        ### side walls
        board_map[0:self.shape[0], 0] = self.map_values["wall"]  # left walls
        board_map[0:self.shape[0], self.shape[1] - 1] = self.map_values["wall"]  # right walls
        board_map[0, 0:self.shape[1]] = self.map_values["wall"]  # up walls
        board_map[self.shape[0] - 1, 0:self.shape[1] - 1] = self.map_values["wall"]  # bottom walls

        ### the home-bases
        # if we play at the top
        if self.home_base_position == (1, 1):
            home_base1 = "home_base"
            home_base2 = "opponent_home_base"
        # if we play at the bottom
        else:
            home_base1 = "opponent_home_base"
            home_base2 = "home_base"
        sym_x, sym_y = self.symmetric((1, 1))
        board_map[1][1] = self.map_values[home_base1]  # the upper home_base
        board_map[sym_x][sym_y] = self.map_values[home_base2]  # the bottom home_base

        """ # Problem: we have no information about num_hob (number of home_base)
        i = 0
        x, y = (1, 1)
        while i < self.num_hob:
            sym_x, sym_y = self.symmetric((x, y))
            board_map[x][y] = self.map_values[home_base1]
            board_map[sym_x][sym_y] = self.map_values[home_base2]

            # next step
            i += 1
            # if we just evaluate a home_base box stuck to the upper limit,
            # the new home_base box will be stuck to the left border
            if x == 1:
                x, y = y+1, 1
            # else the new home_base box will be up and right of the previous one
            else:
                x, y = x-1, y+1"""

        return board_map

    # Return the symmetric of position
    def symmetric(self, position):
        return self.shape[0] - position[0] - 1, position[1]

    # Actualization of the board_map after the use of the radar
    def actualizeMap(self, robot, direction, radar):
        robot_x, robot_y = tuple(robot.position)  # we get the position of the robot
        dir_x, dir_y = self.dir2coord(direction)  # we get the direction (as a tuple)

        # we actualize the board_map with the value of the free_squares
        for i in range(radar.distance):
            x = robot_x + i * dir_x
            y = robot_y + i * dir_y
            sym_x, y = self.symmetric((x, y))  # the symmetric position
            # if we already know there is a free_square, it stores the distance to the home_base
            # so we don't want to overwrite it
            if not type(self.board_map[x][y]) == int:
                self.board_map[x][y] = self.map_values["free_square"]
                self.board_map[sym_x][y] = self.map_values["free_square"]

        # position of the detected object
        x = robot_x + radar.distance * dir_x
        y = robot_y + radar.distance * dir_y
        sym_x, y = self.symmetric((x, y))  # the symmetric position
        # we store the nature of the object in the map (we don't replace immutable items)
        if self.board_map[x][y] not in (self.map_values["home_base"],
                                        self.map_values["opponent_home_base"],
                                        self.map_values["wall"]):
            obj = radar.object
            if obj == "robot":  # if we detected a robot it means there is a free_square at this place
                obj = "free_square"
            if obj is None:  # if we didn't detect, the object is unidentified
                obj = "unidentified"
            self.board_map[x][y] = self.map_values[obj]
            self.board_map[sym_x][y] = self.map_values[obj]
            # during the first step we can identify the other home_base boxes
            if self.current_step == 1:
                self.otherHomeBase((x, y), direction)

        # we update the board_map with the distance to our home_base (from the new discovered free_squares)
        self.distanceMap()

    # Calculate the position of the other home_base boxes (some of them)
    def otherHomeBase(self, position, direction):
        x, y = position
        if 0 < x < self.shape[0] and 0 < y < self.shape[1]:
            if self.home_base_position == (1, 1):
                if direction in ("left", "up"):
                    self.detectedHomeBase(position)
            else:
                if direction in ("left", "down"):
                    self.detectedHomeBase(position)

    # we replace the value of the position with the home_base value and we replace the opponent_home_base with its value
    # we call back detectedHomeBase on the other box we know which are home_base too
    def detectedHomeBase(self, position):
        x, y = position
        sym_x, sym_y = self.symmetric(position)
        self.board_map[x][y] = self.map_values["home_base"]
        self.board_map[sym_x][sym_y] = self.map_values["opponent_home_base"]

        ### we look at the other positions (can be improved !!!)
        (new_x, new_y) = tuple(np.array(position) + np.array(self.dir2coord("left")))  # new position
        if 0 < new_x < self.shape[0] and 0 < new_y < self.shape[1]:
            self.detectedHomeBase((new_x, new_y))
        # we play at the top
        if self.home_base_position == (1, 1):
            (new_x, new_y) = tuple(np.array(position) + np.array(self.dir2coord("up")))  # new position
            if 0 < new_x < self.shape[0] and 0 < new_y < self.shape[1]:
                self.detectedHomeBase((new_x, new_y))
        # we play at the bottom
        else:
            (new_x, new_y) = tuple(np.array(position) + np.array(self.dir2coord("down")))  # new position
            if 0 < new_x < self.shape[0] and 0 < new_y < self.shape[1]:
                self.detectedHomeBase((new_x, new_y))

    # Calculate the distance to the home-base of every point of the board_map
    def distanceMap(self):
        (n, p) = self.shape
        for x in range(1, n - 1):
            for y in range(1, p - 1):
                self.minValue((x, y))

    # return the minimum value of the neighbors + 1, and call minValue on the neighbors which value is > minValue+1
    def minValue(self, position):  # todo: problem with value 0 which is actually '0'
        (x, y) = position
        if self.board_map[x][y] != 0:
            return None
        minValue = np.inf
        # neighbors
        v1 = self.board_map[x - 1][y]  # up
        v2 = self.board_map[x][y - 1]  # left
        v3 = self.board_map[x][y + 1]  # right
        v4 = self.board_map[x + 1][y]  # bottom

        # we look at the neighbors
        for v in [v1, v2, v3, v4]:
            if v < minValue - 1 and v > 0:
                minValue = v + 1  # it takes 1 more step to achieve this position from the home_base
            if v == self.map_values["home_base"]:  # if we are next to the home_base (problem: opponent home_base)
                minValue = 1

        # instantiation of the position's distance
        self.board_map[x][y] = minValue

        # we adjust the value of the distance of the neighbors in the case it has to be
        for (v, pos) in [(v1, (x - 1, y)), (v2, (x, y - 1)), (v3, (x, y + 1)), (v4, (x + 1, y + 1))]:
            if v < np.inf and v > minValue + 1:
                self.minValue(pos)

    # Convert a direction into the corresponding tuple
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
            'right': np.array([1, 0])
        }
        step = switch.get(direction, None)
        if step is None:
            raise ValueError("Not a valid direction ({})".format(direction))
        return tuple(step)

    # Find the best move for one robot
    def bestMove(self, observation, robot):
        position = tuple(robot.position)
        bestMove, value = self.bestMoveRec(observation, position, robot.energy, robot.has_item, self.depth)
        if bestMove == "":  # if no move lead to a finite value in bestMoveRec
            # choose random direction to move
            self.print("random move chosen")
            bestMove = np.random.choice(list(self.directions))
        return bestMove

    # Recursive function to find the best move
    def bestMoveRec(self, observation, position, energy, have_coin, depth):
        # Leaf / we evaluate the board
        if depth == 0:
            x, y = position
            distance_home_base = self.board_map[x][y]
            distance_coin, (coin_x, coin_y) = self.distanceCoin(position)

            # if there is no coin
            delta = -1
            if distance_coin != None:
                delta = 1 / distance_coin * (self.board_map[coin_x][coin_y] < energy - distance_coin)

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
                (x, y) = tuple(np.array(position) + np.array(self.dir2coord(move)))  # new position
                if self.board_map[x][y] == self.map_values["coin"]:  # if we collect a coin
                    have_coin = True
                # we only go to known free_squares
                if self.board_map[x][y] < 0 and not (self.board_map[x][y] == self.map_values["home_base"]) \
                        or self.board_map[x][y] == np.inf:
                    return "", -np.inf
                # recursive call
                bstMv, value = self.bestMoveRec(observation, (x, y), energy - 1 - have_coin, have_coin, depth - 1)
                if value > maxValue:
                    bestMove = move
                    maxValue = value
        return bestMove, value

    # Return the proportion of the board_map that we know
    def knowMap(self):
        (n, p) = self.shape
        prop = 0
        for x in range(1, n - 1):  # we don't look at the borders
            for y in range(1, p - 1):  # we don't look at the borders
                # what with the "unidentified" objects ?
                if self.board_map[x][y] != np.inf:  # if we know what is in the position (x, y)
                    prop += 1
        return prop / (n - 2 * p - 2)  # the dimensions of the board_map without the borders

    # Calculate the shortest distance to a coin
    def distanceCoin(self, position):
        robot_x, robot_y = position
        distance = -1  # distance to the nearest coin
        pos = (-1, -1)  # position of the nearest coin
        dist_map = np.full(self.shape, np.inf)
        dist_map[robot_x][robot_y] = 0
        squares = []

        # look at the neighbors and adjust their distance
        for (x, y) in self.neighbors(position):
            squares.append((x, y))
            dist_map[x][y] = 1

        # searching for the nearest coin
        while distance == -1:
            if len(squares) == 0:
                return None, (-1, -1)
            x, y = squares.pop(0)  # BFS
            dist = dist_map[x][y]
            # if there is a coin
            if self.board_map[x][y] == self.map_values["coin"]:
                distance = dist
                pos = (x, y)
                break
            nghb = self.neighbors((x, y))
            for (i, j) in nghb:
                # if we don't have already visit it
                if dist_map[i][j] == np.inf:
                    squares.append((i, j))
                    dist_map[i][j] = dist + 1

        return distance, pos

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


# Run strategy
if __name__ == "__main__":
    strat3 = Strat3()
    strat3.run()
