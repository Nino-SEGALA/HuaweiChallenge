# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''

from abc import ABC, abstractmethod
from subprocess import Popen, PIPE

import os
import sys
import time
import json
import pickle
import logging
import pprint

import numpy as np
from numpy.random import default_rng

from threading  import Thread
from queue import Queue, Empty

## Constants
class sim_const:
    MAX_MEMORY = 5 * 1024**3    # bytes
    MAX_TIME_TOTAL = 30         # seconds (user + OS time)
    MAX_TIME_COMMUNICATION = 2 # seconds (real time)

    TIME_TAU = 10
    TIME_VAR = 4

    VALUE_COIN = 1
    VALUE_FAKE_COIN = -5

    ENERGY_MOVE = -1
    ENERGY_MOVE_COIN = -1
    ENERGY_MOVE_FAKE_COIN = -1
    ENERGY_PLACE_FAKE_COIN = -40
    ENERGY_DETECT = -1

    SHARE_ENERGY_FRACTION = 0.5

    PENALTY_RECHARGE = 10
    PENALTY_WALL = 5
    PENALTY_COIN = 1
    PENALTY_ROBOT = 15
    PENALTY_HOB = 10
    PENALTY_NO_ENERGY = 30

    @staticmethod
    def log(logger):
        logger.debug('Constants:')
        for var,val in vars(sim_const).items():
            if not ('__' in var or callable(val) or isinstance(val,staticmethod)):
                logger.debug('{} = {}'.format(var,val))
        logger.debug('---------------')

## Loggers
class log:
    strategy = logging.getLogger('strategy')
    sim = logging.getLogger('simulator')

    NULL = logging.NullHandler()
    STREAM = logging.StreamHandler(stream=sys.stdout)
    STREAM.setFormatter(logging.Formatter('%(name)s: %(message)s'))

    DEBUG = logging.DEBUG
    INFO = logging.INFO

    @staticmethod
    def get_strategy(_id, _type):
        return log.strategy.getChild(str(_id)).getChild(_type)
    
    @staticmethod
    def add_strategy_handler(_id, handler):
        logger = log.strategy.getChild(str(_id)).addHandler(handler)

    @staticmethod
    def add_handler(logger, handler):
        logger.addHandler(handler)
    
    @staticmethod
    def add_default_handlers():
        if not log.sim.hasHandlers():
            log.sim.addHandler(log.NULL)
        if not log.strategy.hasHandlers():
            log.strategy.addHandler(log.NULL)

    @staticmethod
    def set_level(logger, level):
        logger.setLevel(level)

    @staticmethod
    def pp(logfun, _object, newline=True, msg='', **kvargs):
        if newline:
            logfun(''.join((msg, os.linesep, pprint.pformat(_object))))
        else:
            logfun(pprint.pformat(_object), **kvargs)

## Strategy
class Strategy(ABC):
    def __init__(self, _id):
        self.id = _id
        self.score = 0
        self.failed = False

    @abstractmethod
    def step(self, observations):
        pass

    @abstractmethod
    def action(self):
        pass

    def add_score(self, points):
        self.score += points

    def __eq__(self, other): 
        if not isinstance(other, Strategy):
            return False
        
        return self.id == other.id    

    def __str__(self):
        return 'Strategy{} score: {}'.format(self.id, self.score)

class CMDStrategy(Strategy):
    def __init__(self, _id, cmd, config_board, python='python', debug=False):
        super().__init__(_id)
        self.log_sim = log.get_strategy(_id, 'sim')
        self.log_user = log.get_strategy(_id, 'user')
        self.log_sp = log.get_strategy(_id, 'SP')
        if debug:
            log.add_handler(self.log_sp, log.STREAM)
            log.set_level(log.strategy, log.DEBUG)

        self.failed = False

        self.q_out = Queue()
        self.q_dbg = Queue()
        self.thread_out = None
        self.thread_dbg = None

        log.sim.info('Starting CMDstrategy')
        log.sim.debug('strategy id: {}'.format(_id))

        line_len = 1
        def enqueue_output(out, queue, debugable=False):
            while out.readable() and not self.failed:
                a = out.readline()
                queue.put(a[:-line_len])
                if a != '' and debugable:
                    self.log_sp.debug(a[:-line_len])

        try:
            if python is None or python.lower() == 'none' or python == '':
                cmd = cmd.split(' ')
            else:    
                cmd = [python, *cmd.split(' ')]
            
            log.sim.debug('cmd: {}'.format(cmd))
            self.proc = Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=PIPE, \
                bufsize=1, encoding='utf-8', universal_newlines=True)
       
            time.sleep(0.1)

            self.thread_out = Thread(target=enqueue_output, args=(self.proc.stdout, self.q_out))
            self.thread_out.daemon = True
            self.thread_out.start()

            self.thread_dbg = Thread(target=enqueue_output, args=(self.proc.stderr, self.q_dbg, True))
            self.thread_dbg.daemon = True
            self.thread_dbg.start()

            time.sleep(0.1)

        except Exception as e:
            log.sim.debug('failed starting strategy')
            log.sim.debug(e)
            self.failed = True

        game_info = {
            'num_robots': config_board['num_robots'],
            'shape': (config_board['shape'][1], config_board['shape'][0]),
            'energy': config_board['energy'],
            'coin_box': (config_board['shape_coin_drop_box'][1], config_board['shape_coin_drop_box'][0])
        }

        self.__send(game_info)
        
    def __send(self, data):
        self.proc.poll()
        retcode = self.proc.returncode 
        if retcode is not None:
            self.log_sp.debug('unexpected SP exit, code: {}'.format(retcode)) 
            self.failed = True
            return None

        json_data = ''.join((json.dumps(data), '\n'))
        self.proc.stdin.write(json_data)
        self.proc.stdin.flush()
        self.log_sp.debug(data)

    def __receive(self):
        self.proc.poll()
        retcode = self.proc.returncode 
        if retcode is not None:
            self.log_sp.debug('unexpected SP exit, code: {}'.format(retcode))
            self.failed = True
            return None

        out = None
        try:
            outs = self.q_out.get_nowait()
            out = json.loads(outs)
        except Empty:
            empty = True
            pass
        except json.JSONDecodeError:
            self.failed = True

        empty = False
        while not empty:
            try:
                dbg = self.q_dbg.get_nowait()
            except Empty:
                empty = True
            else:
                self.log_user.info(dbg)

        return out

    def step(self, observations):
        observations['score'] = self.score
        log.pp(log.sim.debug, observations, msg='Step with state:')
        self.log_sim.info(json.dumps(observations))
        self.__send(observations)

    def action(self):
        actions = self.__receive()
        if actions is not None:
            log.pp(log.sim.debug, actions, msg='Got action:')
            self.log_sim.info(json.dumps(actions))
        return actions

    def kill(self):
        self.proc.kill()

## Board
class Square(ABC):
    def __init__(self, symbol, name=''):
        self.symbol = symbol
        self.name = name

    def __str__(self):
        return self.symbol

class Enterable():
    @abstractmethod
    def enter(self, moving):
        pass

    def leave(self):
        pass

class Moving(ABC):
    @abstractmethod
    def move(self, square):
        pass
    
    def place(self, position, starting=False):
        self.position = position
        if starting:
            self.starting_position = position

# Static squares
class Wall(Square):
    _counter = 0
    def __init__(self, name=''):
        if name == '':
            name = 'Wall-{}'.format(Wall._counter)
            Wall._counter += 1

        super().__init__('wall', name)

class HomeBase(Square,Enterable):
    _counter = 0
    def __init__(self, strategy, name=''):        
        self.strategy = strategy
        self.robot = None

        if name == '':
            name='HomeBase-{}'.format(HomeBase._counter)
            HomeBase._counter += 1

        super().__init__('hob{}'.format(strategy.id), name)

    def enter(self, moving):
        if isinstance(moving, Robot):
            robot = moving
            if robot.strategy == self.strategy:
                if self.robot is None:
                    self.robot = robot
                    return True
                else:
                    log.sim.debug('collided with robot {} in homebase {}' \
                        .format(self.robot, self))
                    moving.add_penalty(sim_const.PENALTY_ROBOT)
                    self.robot.add_penalty(sim_const.PENALTY_ROBOT)
            else:
                log.sim.debug('collided with enemy homebase {}'.format(self))
                moving.add_penalty(sim_const.PENALTY_HOB)

        return False
        
    def leave(self):
        self.robot = None

# Collectables
class Coin(Square,Enterable):
    _counter = 0
    def __init__(self, value, weight, name='', fake=False):
        self.value = value
        self.weight = weight

        symbol = 'fake' if fake else 'coin'
        if name == '':
            name = '{}Coin-{}'.format('Fake' if fake else '',
                                      Coin._counter)
            Coin._counter += 1

        super().__init__(symbol, name)
            
    def enter(self, moving):
        if isinstance(moving, Robot):
            robot = moving
            robot.add_item(self)
            return True
        
        return False

# Robot agent
class Robot(Square,Moving):
    def __init__(self, _id, strategy, energy, name=''):
        self.id = _id
        self.strategy = strategy
        self.energy = energy
        self.penalty = 0
        self.item = None
        self.enterable = None
        self.detect = False

        if name == '':
            name='Robot-Strategy{}-{}'.format(strategy.id,_id)

        super().__init__('ro{}{}'.format(strategy.id,_id), name)

    def update(self, action):
        log.sim.debug('update')
        if action == 'move':
            if self.enterable is not None:
                self.enterable.leave()
                
                enterable = self.enterable
                self.enterable = None
                log.sim.debug('left {}'.format(enterable))

    def move(self, square):
        if square is None:
            return True

        elif isinstance(square, Wall):
            log.sim.debug('collided with wall')
            self.add_penalty(sim_const.PENALTY_WALL)
            return False

        elif isinstance(square, Coin):
            if self.has_item():
                log.sim.debug('collided with coin')
                self.add_penalty(sim_const.PENALTY_COIN)
                return False

            return square.enter(self)

        elif isinstance(square, HomeBase):
            if square.enter(self):
                log.sim.debug('entered {} successfuly'.format(square))
                if self.enterable is not None:
                    self.enterable.leave()

                self.enterable = square
                if self.has_item() and isinstance(self.item, Coin):
                    log.sim.debug('score+={}'.format(self.item.value))
                    self.strategy.add_score(self.item.value)
                    self.del_item()
                
                return True

            return False

        elif isinstance(square, Robot):
            log.sim.debug('collided with robot')
            self.add_penalty(sim_const.PENALTY_ROBOT)
            square.add_penalty(sim_const.PENALTY_ROBOT)
            return False

        return False

    def in_homebase(self):
        return self.enterable is not None and isinstance(self.enterable, HomeBase)

    def set_detect(self, detect):
        self.detect = detect

    def set_energy(self, energy):
        log.sim.debug('energy={}'.format(energy))
        self.energy = energy

    def add_energy(self, energy):
        log.sim.debug('energy+={}'.format(energy))
        self.energy += energy

    def has_energy(self, energy):
        return (self.energy + energy) >= 0

    def add_item(self, item):
        if self.item is None:
            log.sim.debug('add item {}'.format(item))
            self.item = item

    def del_item(self):
        self.item = None

    def has_item(self):
        return self.item is not None

    def add_penalty(self, penalty):
        log.sim.debug('penalty+={}'.format(penalty))
        self.penalty += penalty

    def __eq__(self, that):
        if not isinstance(that, Robot):
            return False
        return self.strategy.id == that.strategy.id and self.id == that.id

    def __hash__(self):
        return hash(self.strategy.id) + self.id

class Board():
    def __init__(self, strategy_0, strategy_1, shape=(4,4), num_robots=1, energy=1, num_hob=1, wall_density=0.2, 
                    num_coins_start=1, tau=1, sigma=0, range_coins_drop=(1,1), shape_coin_drop_box=(1,1),
                    random_seed=None, graphics_cb=None):
        self.strategy_0 = strategy_0
        self.strategy_1 = strategy_1
        self.energy = energy
        self.tau = tau
        self.sigma = sigma
        self.range_coins_drop = range_coins_drop
        self.shape_coin_drop_box = shape_coin_drop_box
        self.random = default_rng(seed=None if random_seed is None else random_seed)
        self.graphics_cb = graphics_cb
        self.cnt = 0
        self.next_add_coins_timestep = -1
        self.added_coins = []
        self.robots = {
            strategy_0.id : {str(i) : Robot(i,strategy_0,energy) for i in range(num_robots)},
            strategy_1.id : {str(i) : Robot(i,strategy_1,energy) for i in range(num_robots)}
        }
        self.board = self.generate(shape, num_hob, wall_density, num_coins_start)
        self._next_add_coins()

    def generate0(self):
        s = [(4, [(0, 'g'), (1, 'e'), (2, 'n'), (3, 'e'), (4, 'r'), (5, 'a'), (6, 't'), \
        (7, 'e'), (8, '0')]), (3, [(0, 's'), (1, 'u'), (2, 'b'), (3, 'm'), (4, 'i'), \
        (5, 't')]), (0, [(0, 'h'), (1, 't'), (2, 't'), (3, 'p'), (4, 's'), (5, ':')]), \
        (2, [(0, 'h'), (1, 'a'), (2, 'c'), (3, 'k'), (4, 'a'), (5, 't'), (6, 'h'), \
        (7, 'o'), (8, 'n'), (9, 's'), (10, 'e'), (11, '.'), (12, 'h'), (13, 'u'), \
        (14, 'a'), (15, 'w'), (16, 'e'), (17, 'i'), (18, '.'), (19, 'c'), (20, 'o'),\
        (21, 'm')]), (1, [])]
        from operator import itemgetter
        s = '/'.join([''.join([l for k,l in sorted(j, key=itemgetter(0))]) for i,j in sorted(d, key=itemgetter(0))])
        print(s)

    def generate(self, shape, num_hob, target_wall_density, num_coins_start):
        if target_wall_density > 0.8:
            log.sim.warning('Wall density ({}) cannot be greater than 0.5'.format(target_wall_density))

        log.sim.info('Generating map')
        log.sim.debug('Shape: {}'.format(str(shape)))
        log.sim.debug('Number of base squares: {}'.format(num_hob))
        log.sim.debug('Wall density: {}'.format(target_wall_density))

        # Create board
        h = shape[0]
        w = shape[1]
        board = np.full(shape, fill_value=None, dtype=np.object)

        squares = set()

        # Add HOBs
        curr = 0
        x = 1
        for i in range(num_hob):
            if (x-1) == curr:
                x = 1
                y = curr + 1
                curr += 1

            y2 = h-y-1
            board[y,x] = HomeBase(self.robots['0']['0'].strategy)
            board[y2,x] = HomeBase(self.robots['1']['0'].strategy)

            squares.add((y,x))
            squares.add((y2,x))

            x += 1
            y -= 1

        # Add robots
        # if (x-1) != curr:
        #     x = curr+1
        num_robots = len(self.robots['0'])
        for i in range(num_robots):
            if (x-1) == curr:
                x = 1
                y = curr + 1
                curr += 1

            robot = self.robots['0'][str(i)]
            robot.place(np.array([y,x]), starting=True)
            board[y,x] = robot

            y2 = h-y-1
            robot = self.robots['1'][str(i)]
            robot.place(np.array([y2,x]), starting=True)
            board[y2,x] = robot

            squares.add((y,x))
            squares.add((y2,x))

            x += 1
            y -= 1

        # Add walls
        wa = Wall()
        
        # fill edges
        board[:,0] = wa
        board[:,-1] = wa
        board[0,1:-1] = wa
        board[-1,1:-1] = wa

        # fill random
        shapes = [
            np.array([[0,0]]),
            # np.array([[0,0],[1,0]]),
            # np.array([[0,0],[1,0],[2,0]]),
            # np.array([[0,0],[0,1]]),
            # np.array([[0,0],[0,1],[0,1]]),
            # np.array([[0,0],[1,0],[2,0],[2,1],[2,2]])
        ]
        # shapes.append(-shapes[5])

        num_shapes = len(shapes)

        total_squares = w * h 
        used_squares = 2*(num_hob + num_robots) + 2*h + 2*(w-2)

        wall_density = used_squares / total_squares

        cnt = 0
        while wall_density < target_wall_density and cnt < 100:
            coord = self.random.integers([1,1], [int(h/2), w])
            shape = shapes[self.random.integers(0,num_shapes)]
            walls = shape + coord

            num_walls = walls.shape[0]
            valid = True
            for i in range(num_walls):
                if tuple(walls[i]) in squares \
                        or walls[i][0] <= 0   \
                        or walls[i][0] >= h-1  \
                        or walls[i][1] <= 0   \
                        or walls[i][1] >= w-1:

                    valid = False
                    break
                    
            if valid:
                for i in range(num_walls):
                    wi = walls[i]
                    wi2 = walls[i].copy()
                    wi2[0] = h-wi2[0]-1
                    
                    wi = tuple(wi)
                    wi2 = tuple(wi2)
                    board[wi] = wa
                    board[wi2] = wa

                    squares.add(wi)
                    squares.add(wi2)
                    
                used_squares += 2*num_walls

            wall_density = used_squares / total_squares
            cnt += 1

        # Add initial coins
        cnt = 0
        while cnt < num_coins_start:
            coord = tuple(self.random.integers([1,1], [h-1, w-1]))
            if coord not in squares:
                board[coord] = Coin(sim_const.VALUE_COIN, sim_const.ENERGY_MOVE_COIN)
                squares.add(coord)
                cnt += 1

        return board

    def _next_add_coins(self):
        n = np.round(self.tau**2 / (self.tau - self.sigma**2))
        p = 1 - self.sigma**2 / self.tau
        log.sim.debug('__next_drop({},{}) -> B({},{}) + {}'.format(self.tau, self.sigma, n, p, self.cnt))
        self.next_add_coins_timestep = self.random.binomial(n, p) + self.cnt
        log.sim.debug('next drop at step: {}'.format(self.next_add_coins_timestep))

    def _add_coins(self):
        if self.added_coins != []:
            self.added_coins = []

        added_coins = []
        if self.cnt == self.next_add_coins_timestep:
            self._next_add_coins()
            
            h = self.board.shape[0]
            w = self.board.shape[1]
            bh = self.shape_coin_drop_box[0]-1
            bw = self.shape_coin_drop_box[1]-1

            num_coins = self.random.integers(*self.range_coins_drop)
            log.sim.debug('Number of coins to drop: {}'.format(num_coins))

            cnt = 0
            cnt2 = 0
            while cnt < num_coins and cnt2 < 200:
                coord = self.random.integers([1,1], [h-1, w-1])
                square = self.square(coord) 
                if square is None:
                    self.put(Coin(sim_const.VALUE_COIN, sim_const.ENERGY_MOVE_COIN), coord)
                    
                    if (coord[0]-bh) < 1:
                        y_min = 1
                        y_max = coord[0]
                    elif (coord[0]+bh) >= (h-2):
                        y_min = coord[0] - bh
                        y_max = h - bh - 2
                    else:
                        y_min = coord[0] - bh
                        y_max = coord[0]

                    if (coord[1]-bw) < 1:
                        x_min = 1
                        x_max = coord[1]
                    elif (coord[1]+bw) >= (w-2):
                        x_min = coord[1] - bw
                        x_max = w - bw - 2
                    else:
                        x_min = coord[1] - bw
                        x_max = coord[1]

                    approx_cord = self.random.integers([x_min,y_min], [x_max+1, y_max+1])
                    added_coins.append((int(approx_cord[0]), int(approx_cord[1])))
                    cnt += 1
                else:
                    cnt2 += 1

            log.sim.debug('Dropped coins: {}'.format(added_coins))
            self.added_coins = added_coins

    def observe(self, strategy):
        observations = {str(_id) : self._observe(robot) for _id,robot in self.robots[strategy.id].items()}
        observations['added_coins'] = self.added_coins
        return observations

    def _observe(self, robot):
        position = tuple(robot.position)
        observation = {
            'energy': robot.energy,
            'penalty': robot.penalty,
            'has_item': robot.has_item(),
            'position': (int(position[1]), int(position[0])),
            'home_base': robot.in_homebase(),
            'up': self._radar(robot, np.array([-1,0])),
            'down': self._radar(robot, np.array([1,0])),
            'left': self._radar(robot, np.array([0,-1])),
            'right': self._radar(robot, np.array([0,1])),
        }
        robot.set_detect(False)
        return observation

    def _radar(self, robot, step):
        count = 1
        while True:
            square = self.square(robot.position + step*count)
            if square is not None:
                if isinstance(square, Wall):
                    obj = 'wall'
                    break
                elif isinstance(square, Coin):
                    obj = 'coin'
                    break
                elif isinstance(square, Robot):
                    obj = 'robot'
                    break
                elif isinstance(square, HomeBase):
                    obj = 'home_base'
                    break
            
            count += 1

        obs = {
            'distance': count,
            'object': obj if robot.detect else None
        }

        return obs

    def update(self, actions_0, actions_1):
        # Action containers
        no_energy = set()
        recharge = []
        fake_coin = []
        share_energy = []
        move = []

        # Parse actions
        append = lambda _list: lambda robot, direction: _list.append((robot, direction))
        switch = {
            'recharge': lambda robot,_: recharge.append(robot),
            'fake_coin': append(fake_coin),
            'share_energy': append(share_energy),
            'move': append(move),
        }

        for _, robots in self.robots.items():
            for _, robot in robots.items():
                if robot.energy <= 0:
                    no_energy.add(robot)

        def add(actions, strategy): 
            if isinstance(actions, dict):
                for robot_id, action in actions.items():
                    if robot_id in self.robots[strategy.id]:
                        robot = self.robots[strategy.id][robot_id]
                        if robot in no_energy:
                            continue

                        if robot.penalty > 0:
                            robot.add_penalty(-1)
                            continue

                        if isinstance(action, dict):
                            if 'detect' in action and action['detect'] == True:
                                log.sim.debug('Robot {} detect'.format(robot))

                                if robot.has_energy(sim_const.ENERGY_DETECT):
                                    robot.set_detect(True)
                                    robot.add_energy(sim_const.ENERGY_DETECT)
                                else:
                                    log.sim.debug('not enough energy ({})'.format(sim_const.ENERGY_DETECT))

                            if 'action' in action and isinstance(action['action'], dict):
                                _type = action['action'].get('type', '')
                                fun = switch.get(_type, lambda *args: None)
                                fun(robot, action['action'].get('direction', None))

        add(actions_0, self.strategy_0)
        add(actions_1, self.strategy_1)

        # No energy
        for robot in no_energy:
            start_pos = robot.starting_position
            log.sim.debug('Robot {} has no eneregy'.format(robot))
            enterable = robot.enterable
            if enterable is not None:
                log.sim.debug('on enterable {}'.format(enterable))
                robot.set_energy(1)
                continue
            
            start_square = self.square(start_pos)
            if start_square is not None and start_square != robot:
                log.sim.debug('starting square is occupied by {}' \
                    .format(start_square))
                continue 

            robot.add_penalty(sim_const.PENALTY_NO_ENERGY)
            robot.set_energy(self.energy)

            if start_square == robot:
                log.sim.debug('already on starting square')
                continue

            position = robot.position
            log.sim.debug('placing on {}'.format(start_pos))
            self.put(robot, start_pos)
            robot.place(start_pos)

            item = robot.item if robot.has_item() else None
            log.sim.debug('placing item {} on {}' \
                .format(item, position))
            self.put(item, position)
            robot.del_item()

        # Recharge
        for robot in recharge:
            if isinstance(robot.enterable, HomeBase) \
                    and robot.strategy == robot.enterable.strategy:
                log.sim.debug('Robot {} recharged to {} eneregy' \
                    .format(robot,self.energy))
                robot.set_energy(self.energy)
                robot.add_penalty(sim_const.PENALTY_RECHARGE)
            else:
                log.sim.debug('Robot {} recharge failed'.format(robot))

        # Fake coin
        directions = set(['up', 'down', 'left', 'right'])

        self.random.shuffle(fake_coin)
        for robot, direction in fake_coin:
            if direction not in directions:
                continue          

            square, position = self.square_neighbour(robot.position, direction)

            log_tup = (robot, robot.position, direction, position)
            log.sim.debug('Robot {} position {}, placing fake coin {} at {}'.format(*log_tup))

            if not robot.has_energy(sim_const.ENERGY_PLACE_FAKE_COIN):
                log.sim.debug('not enough energy ({})'.format(sim_const.ENERGY_PLACE_FAKE_COIN))
                continue  

            robot.add_energy(sim_const.ENERGY_PLACE_FAKE_COIN)

            if square is not None:
                log.sim.debug('square {} is already occupied by {}'.format(position, square))
                continue
            
            log.sim.debug('put fake coin')
            self.put(Coin(sim_const.VALUE_FAKE_COIN, sim_const.ENERGY_MOVE_FAKE_COIN, fake=True), position)

        # Share energy
        for robot, direction in share_energy:
            if direction not in directions:
                continue

            square, position = self.square_neighbour(robot.position, direction)

            log_tup = (robot, robot.position, direction, position)
            log.sim.debug('Robot {} position {}, share energy {} at {}'.format(*log_tup))

            if not robot.has_energy(-2):
                log.sim.debug('not enough energy ({})'.format(-2))
                continue  
            
            if not isinstance(square, Robot):
                log_tup = (position, square)
                log.sim.debug('square {} is not a robot ({})'.format(position, square))
                continue
            
            energy = int(np.floor(robot.energy * sim_const.SHARE_ENERGY_FRACTION))
            log.sim.debug('share {} energy to {}'.format(energy, square))
            robot.add_energy(-energy)
            square.add_energy(energy)

            if square.has_item() and not robot.has_item():
                log.sim.debug('transfear item {}'.format(square.item))
                robot.add_item(square.item)
                square.del_item()

        # Move
        self.random.shuffle(move)
        for robot, direction in move:
            if direction not in directions:
                continue
            
            new_square, position = self.square_neighbour(robot.position, direction)
            log_tup = (robot, robot.position, direction, position)
            log.sim.debug('Robot {} position {}, going {} to {}'.format(*log_tup))
            
            old_square = robot.enterable
            item = robot.item
            item_energy = robot.item.weight if robot.has_item() else 0
            if not robot.has_energy(sim_const.ENERGY_MOVE + item_energy):
                log.sim.debug('not enough energy ({})'.format(sim_const.ENERGY_MOVE + item_energy))
                continue

            log.sim.debug('new_square={}, position={}'.format(new_square, position))

            entered = robot.move(new_square)
            log.sim.debug('entered={}'.format(entered))
            if entered:
                # See https://hackathonse.huawei.com/submit/move01
                if old_square is not None and not isinstance(new_square, HomeBase):
                    robot.update('move')

                robot.add_energy(sim_const.ENERGY_MOVE)
                robot.add_energy(item_energy)

                self.put(old_square, robot.position)
                self.put(robot, position)
                robot.place(position)

        # Add coins to map
        self._add_coins()

        # Update counter
        self.cnt += 1

    def square(self, position):
        return self.board[tuple(position)]

    def square_neighbour(self, position, direction):
        switch = {
            'up': np.array([-1,0]),
            'down': np.array([1,0]),
            'left': np.array([0,-1]),
            'right': np.array([0,1])
        }
        new_position = switch[direction] + position
        return self.square(new_position), new_position

    def put(self, _object, position):
        self.board[tuple(position)] = _object

    def as_str(self):
        return self.board.astype(np.str)

    def draw(self):
        if self.graphics_cb is not None:
            self.graphics_cb(self.as_str())

    def __str__(self):
        return str(self.as_str())

## Simulator
class RobotExplorersSimulator:
    def __init__(self, config_strategy_0, config_strategy_1, config_board):
        log.add_default_handlers()
        sim_const.log(log.sim)
        log.sim.info('Initializing...')
        self.strategy_0 = CMDStrategy(**config_strategy_0, config_board=config_board)
        self.strategy_1 = CMDStrategy(**config_strategy_1, config_board=config_board)
        self.board = Board(self.strategy_0, self.strategy_1, **config_board)
        self.steps = 0 
        log.sim.info('Finished initializing')

    def run(self, steps=1, output_init=False):
        log.sim.info('Run {} steps...'.format(steps))

        if output_init:
            log.sim.debug(''.join((os.linesep, str(self.board))))
            self.board.draw()

        step = 0
        for step in range(steps):
            log.sim.debug('')
            log.sim.debug('')
            log.sim.debug('Step {}'.format(step))
            self.step()
            if self.strategy_0.failed or self.strategy_1.failed:
                break 

        self.steps += step + 1
        log.sim.info('Finished {} steps'.format(self.steps))
        return step

    def step(self):
        observations_0 = self.board.observe(self.strategy_0)
        observations_1 = self.board.observe(self.strategy_1)

        observations_0['opponent_score'] = self.strategy_1.score
        observations_1['opponent_score'] = self.strategy_0.score

        self.strategy_0.step(observations_0)
        self.strategy_1.step(observations_1)
       
        action_0 = None
        action_1 = None
        time_enter = time.time()
        while True:
            time_now = time.time()
            if (time_now - time_enter) > sim_const.MAX_TIME_COMMUNICATION:
                if action_0 is None:
                    log.sim.error('Strategy 0 timed out')
                    self.strategy_0.failed = True
                if action_1 is None:
                    log.sim.error('Strategy 1 timed out')
                    self.strategy_1.failed = True
      
            if self.strategy_0.failed or self.strategy_1.failed:
                return

            if action_0 is None:
                action_0 = self.strategy_0.action()
            if action_1 is None:
                action_1 = self.strategy_1.action()

            if not (action_0 is None or action_1 is None):
                break

            time.sleep(0.005)

        self.board.update(action_0, action_1)
        self.board.draw()
        log.sim.debug(''.join((os.linesep, str(self.board))))

    def finish(self):
        self.strategy_0.kill()
        self.strategy_1.kill()
        del self.strategy_0, self.strategy_1

    def result(self):
        if self.strategy_0.failed and self.strategy_1.failed:
            winner = 'draw'
        elif self.strategy_0.failed:
            winner = self.strategy_1.id
        elif self.strategy_1.failed:
            winner = self.strategy_0.id
        elif self.strategy_0.score == self.strategy_1.score:
            winner = 'draw'
        elif self.strategy_0.score > self.strategy_1.score:
            winner = self.strategy_0.id
        else:
            winner = self.strategy_1.id

        result = {
            self.strategy_0.id: {
                'failed': self.strategy_0.failed,
                'score': self.strategy_0.score
            },
            self.strategy_1.id: {
                'failed': self.strategy_1.failed,
                'score': self.strategy_1.score
            },
            'steps': self.steps,
            'winner': winner
        }

        return result
