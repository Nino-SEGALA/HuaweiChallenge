# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''
from .strategy import Strategy
from .simulator import RobotExplorersSimulator as Simulator
from .simulator import log
__all__ = ['Simulator','Strategy','log']