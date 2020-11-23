# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Romain Deffayet
    Karl Gäfvert
'''

import numpy as np
from robot_explorers import Strategy

class Random(Strategy):
    '''
    Implementation of an simple strategy based on random movement
    (does not follow optimal path to coin even when in direct view)

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
        # Keep track of past steps
        self.path = {r:[] for r in range(self.num_robots)}


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
        
        # Initialize empty action
        action = self.action()
        
        # Used to prevent collisions among own robots
        moves = set()
        
        # Evaluation of move actions are random
        # Prevent robots from moving to each others current posistion
        for robot_id in range(self.num_robots):
            moves.add(tuple(observation.robot(robot_id).position))

        # Loop over robots
        for robot_id in range(self.num_robots):

            # Get robot specific observation
            robot = observation.robot(robot_id)
            position = tuple(robot.position)

            self.print('')
            self.print('Robot {}:'.format(robot_id))
            self.print('position: {}'.format(position))
            self.print('energy: {}'.format(robot.energy))

            # Enable object detection
            action.detect(robot_id)
            
            ### Out of energy, teleported to start position
            if robot.energy == 0:
                self.print('teleported')
                self.path[robot_id] = []    # reset path
                continue

            ### Robot is penalized, do nothing
            if robot.penalty > 0:
                self.print('penalty: {}'.format(robot.penalty))
                continue

            path_len = len(self.path[robot_id])
            self.print('path len: {}'.format(path_len))
            
            ### In home base and low on eneregy
            if robot.home_base and robot.energy < (self.max_energy/3):
                self.print('recharge') 
                action.recharge(robot_id)                               # recharge

            ### When energy is too low or coin is found, go back to base
            elif path_len > 0 and (path_len >= (robot.energy-10) or robot.has_item):
                # True if no collision happened last move
                if self.path[robot_id][-1] == position:
                    self.path[robot_id].pop()                           # remove previous move
                
                # true if haven't arrived at start position yet
                if len(self.path[robot_id]) > 0:
                    
                    # select the direction needed to go back
                    next_pos = self.path[robot_id][-1]

                    # check if next square is already occupued
                    if next_pos not in moves:
                        self.print('pos: {}, next_pos: {}'.format(position, next_pos))
                        # Get direction of next move
                        direction = self.coord2dir(position, next_pos)
                        action.move(robot_id, direction)                # set move action
                        moves.add(next_pos)
            
            ### Go to base and drop coin
            elif path_len == 0 and robot.has_item:
                for direction in self.directions:
                    radar = observation.radar(robot_id, direction)      # get radar
                    
                    # Get target coordinates of next move
                    next_pos = self.dir2coord(position, direction)
                    
                    # check if next square is already occupued
                    if next_pos in moves:
                        continue
                    
                    if radar.object == 'home_base':
                        action.move(robot_id, direction)                # set move action
                        moves.add(next_pos)
                        break
            
            ### Otherwise, move in random direction
            else:
                directions = set(self.directions)
                cnt = 0
                while cnt < 4:
                    cnt += 1

                    # choose random direction to move
                    direction = np.random.choice(list(directions))
                    radar = observation.radar(robot_id, direction)      # get radar             
                    
                    # Get target coordinates of next move
                    next_pos = self.dir2coord(position, direction)
                    
                    # check if next square is already occupued
                    if next_pos in moves:
                        continue

                    # Avoid simple collisions, get coin if next to robot
                    if (radar.distance > 1 or radar.object == 'coin'):
                        self.path[robot_id].append(position)            # add current position to path
                        action.move(robot_id, direction)                # set move action
                        moves.add(next_pos)
                        self.print('add: {}'.format(next_pos))
                        break
                    
                    directions.discard(direction)

            self.print('moves: {}'.format(moves))

        return action


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
            raise ValueError("Starting position and goal are not adjacent")


    def dir2coord(self, pos, direction):
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
            'up': np.array([0,-1]),
            'down': np.array([0,1]),
            'left': np.array([-1,0]),
            'right': np.array([1,0])
        }
        step = switch.get(direction, None)
        if step is None:
            raise ValueError("Not a valid direction ({})".format(direction))
        return tuple(np.array(pos) + step)


# Run strategy
if __name__ == "__main__":
    random = Random()
    random.run()
