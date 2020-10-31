# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Romain Deffayet
    Karl Gäfvert
'''

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

def create_grid(N, M, ax):
    ax.set_xlim(0,N)
    ax.set_ylim(0,M)

    ax.set_aspect('equal')
    
    ax.grid(color = 'k')
    
    ax.axes.get_xaxis().set_ticks(np.arange(N))
    ax.axes.get_yaxis().set_ticks(np.arange(M))
    ax.axes.get_xaxis().set_ticklabels([])
    ax.axes.get_yaxis().set_ticklabels([])

def fill_rect(x, y, col, ax):
    ax.fill([x, x+1, x+1, x], [y, y, y+1, y+1], col)
    
def fill_circle(x,y, col, ax):
    circle1 = Circle((x + .5, y + .5), 0.25, color=col)
    circle2 = Circle((x + .5, y + .5), 0.25, color='k', fill = False, linewidth=2)
    ax.add_artist(circle1)
    ax.add_artist(circle2)

def add_agent(x, y, team, ax):
    #### Head
    if team == '0':
        r_head = Rectangle((x + .25, y + .15), .5, .5, color = 'indianred')
    else:
        r_head = Rectangle((x + .25, y + .15), .5, .5, color = 'cornflowerblue')
    r_head_border = Rectangle((x + .25, y + .15), .5, .5, color = 'k', fill = False, linewidth=2)

    ax.add_artist(r_head)
    ax.add_artist(r_head_border)
    
    #### Arms
    ax.plot([x+.15, x + .3], [y+.4, y+.4], color = 'k', linewidth=2)
    ax.plot([x+.7, x + .85], [y+.4, y+.4], color = 'k', linewidth=2)
    
    ax.plot([x+.15, x + .15], [y+.4, y+.55], color = 'k', linewidth=2)
    ax.plot([x+.85, x + .85], [y+.4, y+.55], color = 'k', linewidth=2)
    
    ### Eyes
    eye1 = Circle((x + .35, y + .5), 0.05, color='k')
    eye2 = Circle((x + .65, y + .5), 0.05, color='k')

    ax.add_artist(eye1)
    ax.add_artist(eye2)
    
    ### Mouth
    r_mouth = Rectangle((x + .35, y + .25), .3, .1, color = 'w')
    r_mouth_border = Rectangle((x + .35, y + .25), .3, .1, color = 'k', fill=False, linewidth=2)
    ax.add_artist(r_mouth)
    ax.add_artist(r_mouth_border)

def draw_energy(x, y, amount, max_energy, ax):
    if amount > max_energy:
        amount = max_energy
    r1 = Rectangle((x + .75 - amount/max_energy * .5, y + .8), amount/max_energy * .5, .1, color = 'orange')
    r2 = Rectangle((x + .25, y + .8), .5, .1, color = 'k', fill = False, linewidth=2)
    ax.add_artist(r1)
    ax.add_artist(r2)

def render(game_state, obs_0, obs_1, max_energy, ax):    #### Game state is a 2D string array
    ax.clear()
    N, M = game_state.shape
    create_grid(M,N, ax)
    
    for (y,x), el in np.ndenumerate(game_state):
        y = N-y-1 # Invert y to match game coordinate system
        if el[:3] == 'hob':
            if el[3:] == '0':
                fill_rect(x,y, 'indianred', ax)
            else:
                fill_rect(x,y, 'cornflowerblue', ax)
        elif el[:2] == 'ro':
            add_agent(x,y,el[2], ax)
            obs = obs_0 if el[2] == '0' else obs_1
            if obs_0 is not None and obs_1 is not None:
                draw_energy(x, y, obs[el[3]]['energy'], max_energy, ax)
            else:
                draw_energy(x, y, max_energy, max_energy, ax)
        elif el == 'wall':
            fill_rect(x,y,'k', ax)
        elif el == 'coin':
            fill_circle(x,y,'yellow', ax)
        elif el == 'fake':
            fill_circle(x,y,'olivedrab', ax)
