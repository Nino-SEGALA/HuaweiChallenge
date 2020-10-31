# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''

import argparse
from .gui import GUI

parser = argparse.ArgumentParser(description='Run GUI')
parser.add_argument('input_dir', metavar='input_dir', type=str, help='Path to saved simulation. Ex. "results/10212020_021804"')
parser.add_argument('--fps', type=int, default='2', help='FPS during visualization')
parser.add_argument('--silent-strategy-0', action='store_true', help='Disable strategy 0 simulator output')
parser.add_argument('--silent-strategy-1', action='store_true', help='Disable strategy 1 simulator output')
parser.add_argument('--about', action='store_true', help='Print info and license')

# Args
args = parser.parse_args()

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
    Romain Deffayet
''')
    exit(0)

gui = GUI(disable_0=args.silent_strategy_0, disable_1=args.silent_strategy_1)
gui.play_from_file(args.input_dir, fps=args.fps)
