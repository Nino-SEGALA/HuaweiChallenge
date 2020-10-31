# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''

import sys
import os
import json
import time

from abc import ABC, abstractmethod
from threading  import Thread
from queue import Queue, Empty
from types import SimpleNamespace

class Strategy:
    def __init__(self):
        super().__init__()

        self.running = True
        self.q_in = Queue()
        self.thread_in = None

        line_len = len(os.linesep)
        def enqueue_output(out, queue):
            while out.readable() and self.running:
                queue.put(out.readline().decode('utf-8')[:-line_len])

        self.thread_in = Thread(target=enqueue_output, args=(sys.stdin.buffer, self.q_in))
        self.thread_in.daemon = True
        self.thread_in.start()

        time.sleep(0.1)

        self.__setup()
        self.init()

    def __setup(self):
        game_info = None
        while game_info is None:
            game_info = self.__receive()
        self.num_robots = game_info['num_robots']
        self.shape = game_info['shape']
        self.max_energy = game_info['energy']
        self.coin_box = game_info['coin_box']

    def __observe(self):
        observations = None
        while observations is None:
            observations = self.__receive()
        return Observation(observations)

    def __action(self, actions):
        self.__send(actions.get())

    def __send(self, data):
        encoded = ''.join((json.dumps(data), '\n')).encode('utf-8')
        sys.stdout.buffer.write(encoded)
        sys.stdout.buffer.flush()

    def __receive(self, block=True):          
        try:
            json_data = self.q_in.get_nowait()
        except Empty:
            pass
        else:
            data = json.loads(json_data)
            return data
        
        return None

    def run(self):
        while self.running:
            observations = self.__observe()
            actions = self.step(observations)
            self.__action(actions)

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def step(self, observations):
        pass

    def print(self, *objects, sep=' ', **kvargs):
        output = ''.join((sep.join([str(obj) for obj in objects]), '\n'))
        sys.stderr.buffer.write(output.encode('utf-8'))
        sys.stderr.flush()

    def action(self):
        return Action(self.num_robots)

class Observation:
    def __init__(self, observations):
        self.observations = observations
        self.score = observations['score']
        self.opponent_score = observations['opponent_score']
        self.added_coins = observations['added_coins']
        del observations['score'], observations['opponent_score'], observations['added_coins']
        
        observations_transform = {}
        for r,obs in observations.items():
            obs_transform = {}
            obs_transform['id'] = r
            for k,v in obs.items():
                if isinstance(v, dict):
                    obs_transform[k] = SimpleNamespace(**v)
                else:
                    obs_transform[k] = v
            observations_transform[r] = obs_transform

        self.__robots = {str(r) : SimpleNamespace(**obs)
            for r,obs in observations_transform.items()}

        self.__directions = set(['up', 'down', 'left', 'right', None])
    
    def __is_robot(self, robot):
        return str(robot) in self.__robots

    def __is_direction(self, direction):
        return direction in self.__directions

    def robot(self, robot):
        return self.__robots[str(robot)] if self.__is_robot(robot) else None

    def radar(self, robot, direction):
        if self.__is_robot(robot) and self.__is_direction(direction):
            return SimpleNamespace(**self.observations[str(robot)][direction])

        return None

class Action:
    def __init__(self, num_robots):
        self.__robots = {str(i) : {
            'action': {
                'type': None,
                'direction': None
            },
            'detect': False
        } for i in range(num_robots)}
        self.__directions = set(['up', 'down', 'left', 'right', None])

    def get(self):
        return self.__robots

    def __is_robot(self, robot):
        return str(robot) in self.__robots

    def __is_direction(self, direction):
        return direction in self.__directions

    def __set_action(self, robot, action, direction):
        if not (self.__is_robot(robot) and self.__is_direction(direction)):
            return False

        robot = str(robot)
        self.__robots[robot]['action']['type'] = action
        self.__robots[robot]['action']['direction'] = direction
        
        return True

    def detect(self, robot):
        if not self.__is_robot(robot):
            return False
        
        self.__robots[str(robot)]['detect'] = True
        
        return True

    def move(self, robot, direction):
        return self.__set_action(robot, 'move', direction)

    def fake_coin(self, robot, direction):
        return self.__set_action(robot, 'fake_coin', direction)

    def share_energy(self, robot, direction):
        return self.__set_action(robot, 'share_energy', direction)

    def recharge(self, robot):
        return self.__set_action(robot, 'recharge', None)

    def none(self, robot):
        return self.__set_action(robot, None, None)
