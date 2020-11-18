# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''

import argparse
from .simulator import RobotExplorersSimulator, log

## CMD arguments
parser = argparse.ArgumentParser(description='Run simulator')
parser.add_argument('strategy_0', metavar='strategy0', type=str, help='Path to strategy file. Ex: "./basic_strategy.py"')
parser.add_argument('strategy_1', metavar='strategy1', type=str, help='Path to strategy file. Ex: "./my_strategy.py"')
parser.add_argument('--timesteps', type=int, default='1', help='Number of timesteps to run')
parser.add_argument('--visualize', action='store_true', help='Visualize simulation in display window')
parser.add_argument('--fps', type=int, default='2', help='FPS during visualization')
parser.add_argument('--headless', action='store_true', help='Save simulation visualizations and debug info to disk')
parser.add_argument('--output-dir', type=str, default='results', help='Output directory. Ex. "./output"')
parser.add_argument('--silent-strategy-0', action='store_true', help='Disable strategy 0 simulator output')
parser.add_argument('--silent-strategy-1', action='store_true', help='Disable strategy 1 simulator output')
parser.add_argument('--debug-strategy', action='store_true', help='Print debug information for strategies')
parser.add_argument('--debug-sim', action='store_true', help='Print debug information for simulator')
parser.add_argument('--about', action='store_true', help='Print info and license')


## Args
args = parser.parse_args()
max_energy = 700

# Print about
if args.about:
    print('''Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

    1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above 
        copyright notice, this list of conditions and the following 
        disclaimer in the documentation and/or other materials provided 
        with the distribution.

    3. Neither the name of the copyright holder nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors:
    Karl Gäfvert
''')
    exit(0)

## GUI
if args.visualize or args.headless:
    from .gui import GUI
    gui = GUI(max_energy=max_energy, disable_0=args.silent_strategy_0, disable_1=args.silent_strategy_1, disable_print=args.headless)

## Config
config_strategy_0 = {'_id': '0', 'cmd': args.strategy_0, 'debug': (args.debug_strategy and not args.silent_strategy_0)}
config_strategy_1 = {'_id': '1', 'cmd': args.strategy_1, 'debug': (args.debug_strategy and not args.silent_strategy_1)}

graphics_cb = gui.draw if args.visualize or args.headless else None
# config_board = {
#     'shape': (12, 12),
#     'num_robots': 3,
#     'energy': max_energy,
#     'num_hob': 3,
#     'wall_density': 0.55,
#     'num_coins_start': 10,
#     'tau': 15,
#     'sigma': 3,
#     'range_coins_drop': (0, 2),
#     'shape_coin_drop_box': (3, 3),
#     'random_seed': 11111,
#     'graphics_cb': graphics_cb
# }

config_board = {
    'shape': (20, 25), # height, width
    'num_robots': 2,
    'energy': max_energy,
    'num_hob': 5,
    'wall_density': 0.40,
    'num_coins_start': 35,
    'tau': 30,
    'sigma': 4,
    'range_coins_drop': (1, 4),
    'shape_coin_drop_box': (3, 4),
    'random_seed': 11111,
    'graphics_cb': graphics_cb
}

## Logger
log.add_handler(log.sim, log.STREAM)
log.set_level(log.sim, log.DEBUG if args.debug_sim else log.INFO)

if args.silent_strategy_0 and args.silent_strategy_1:
    log.add_handler(log.strategy, log.NULL)
log.set_level(log.strategy, log.INFO)

## Log args
log.sim.debug('Arguments:')
for arg,val in vars(args).items():
    log.sim.debug('{} = {}'.format(arg,val))
log.sim.debug('---------------')

## Simulation 
sim = RobotExplorersSimulator(config_strategy_0, config_strategy_1, config_board)

sim.run(steps=args.timesteps, output_init=True)
print(sim.result())

# sim.finish()
# del sim

if args.visualize:
    gui.play(fps=args.fps)

if args.headless:
    gui.save(args.output_dir)