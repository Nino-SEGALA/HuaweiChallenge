# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Romain Deffayet
    Karl Gäfvert
'''

import sys
import logging
import json
import os.path as path
from os import mkdir
from datetime import datetime

from videofig import videofig
import matplotlib.pyplot as plt

from .gridworld import render
from ..simulator import log

class SimulatorOutputHandler(logging.Handler):
    def __init__(self, storage):
        super().__init__()
        self.storage = storage

    def emit(self, record):
        if len(self.storage) > 0:
            try:
                message = json.loads(record.getMessage())
            except:
                message = record.getMessage()

            self.storage[-1].append(message)

class UserOutputHandler(logging.Handler):
    def __init__(self, storage):
        super().__init__()
        self.storage = storage

    def emit(self, record):
        if len(self.storage) > 0:
            self.storage[-1].append(record.getMessage())

class GUI:
    def __init__(self, max_energy=100, no_output_init=False, disable_0=False, disable_1=False, disable_print=False):
        self.max_energy = max_energy
        self.disable_0 = disable_0
        self.disable_1 = disable_1

        self.log = logging.getLogger("gui")
        sh = logging.StreamHandler(stream=sys.stdout)
        # sh.setFormatter(logging.Formatter('%(name)s: %(message)s'))
        sh.setFormatter(logging.Formatter(' %(message)s'))
        self.log.addHandler(logging.NullHandler() if disable_print else sh)
        self.log.setLevel(logging.INFO)

        self.history = []

        self.out_strategy0 = []
        self.out_strategy1 = []
        
        self.debug_strategy0 = []
        self.debug_strategy1 = []

        self.__update_lists()
        if no_output_init:
            self.__update_lists()

        log.get_strategy(0, 'sim').addHandler(SimulatorOutputHandler(self.out_strategy0))
        log.get_strategy(1, 'sim').addHandler(SimulatorOutputHandler(self.out_strategy1))
        log.get_strategy(0, 'user').addHandler(UserOutputHandler(self.debug_strategy0))
        log.get_strategy(1, 'user').addHandler(UserOutputHandler(self.debug_strategy1))

    def __update_lists(self):
        self.out_strategy0.append([])
        self.out_strategy1.append([])
        self.debug_strategy0.append([])
        self.debug_strategy1.append([])
        
    def draw(self, map_state):
        self.history.append(map_state)
        self.__update_lists()

    def __frames_total(self):
        return len(self.history)

    def __redraw_fn_factory(self, is_img=False):
        def redraw_fn(frame, axes):
            observations_0 = None
            observations_1 = None
            
            self.log.info("----------------------- Frame {} -----------------------".format(frame)) 

            if isinstance(self.out_strategy0[frame], list) \
                    and len(self.out_strategy0[frame]) >= 2:
                
                observations_0 = self.out_strategy0[frame][0]
                actions = self.out_strategy0[frame][1]
                if not self.disable_0:
                    self.log.info("")
                    self.log.info("STRATEGY 0")
                    
                    self.log.info("")
                    self.log.info("State:")
                    self.log.info("------------")
                    log.pp(self.log.info, observations_0, newline=False)
                    
                    self.log.info("")
                    self.log.info("Action:")
                    self.log.info("---------------")
                    log.pp(self.log.info, actions, newline=False)
                    
                    self.log.info("")
                    self.log.info("Debug output: ")
                    self.log.info("---------------")
                    for message in self.debug_strategy0[frame]:
                        self.log.info(message)
                    
                    self.log.info("")

                observations_1 = self.out_strategy1[frame][0]
                actions = self.out_strategy1[frame][1]
                if not self.disable_1:
                    self.log.info("")
                    self.log.info("STRATEGY 1")
                    
                    self.log.info("")
                    self.log.info("State:")
                    self.log.info("---------------")
                    log.pp(self.log.info, observations_1, newline=False)
                    
                    self.log.info("")
                    self.log.info("Action:")
                    self.log.info("---------------")
                    log.pp(self.log.info, actions, newline=False)
                    
                    self.log.info("")
                    self.log.info("Debug output: ")
                    self.log.info("---------------")
                    for message in self.debug_strategy1[frame]:
                        self.log.info(message)

                    self.log.info("")

            self.log.info("")

            if not is_img:
                render(self.history[frame], observations_0, observations_1, self.max_energy, axes)
            else:
                axes.imshow(self.history[frame])                

        return redraw_fn

    # Play game history in window
    def play(self, fps=1):
        videofig(self.__frames_total(), self.__redraw_fn_factory(), play_fps=fps, figname="Hackathon")
    
    # Save game history
    def save(self, output_dir):
        if not path.isdir(output_dir):
            print('Output path does not exist ({})'.format(output_dir))
            exit(-1)
        
        now = datetime.now()
        output_dir = path.join(output_dir, datetime.now().strftime("%m%d%Y_%H%M%S"))
        mkdir(output_dir)

        videofig(self.__frames_total(), self.__redraw_fn_factory(), figname="Hackathon", save_dir=output_dir)

        data = {
            'n_frames': self.__frames_total(),
            's0_out': self.out_strategy0,
            's1_out': self.out_strategy1,
            's0_debug': self.debug_strategy0,
            's1_debug': self.debug_strategy1
        }

        with open(path.join(output_dir, "data.json"), 'w') as f:
            json.dump(data, f)

    def play_from_file(self, input_dir, fps=1):
        with open(path.join(input_dir, "data.json"), 'r') as f:
            data = json.load(f)

        self.history = [plt.imread(path.join(input_dir, '{:04d}.jpg'.format(i))) for i in range(data['n_frames'])]
        self.out_strategy0 = data['s0_out']
        self.out_strategy1 = data['s1_out']
        self.debug_strategy0 = data['s0_debug']
        self.debug_strategy1 = data['s1_debug']
        
        videofig(data['n_frames'], self.__redraw_fn_factory(True), play_fps=fps, figname="Hackathon")
