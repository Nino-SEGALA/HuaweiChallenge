# -*- coding: utf-8 -*-
'''
Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.

Authors:
    Karl Gäfvert
'''

from robot_explorers import Strategy

class Test(Strategy):
    '''Demo of strategy, observation, and action APIs'''

    def __init__(self):
        super().__init__()


    def init(self):
        '''Called once before the first observation has been received.'''

        self.print('--- strategy init ---')
        self.print('num_robots: {}'.format(self.num_robots))
        self.print('shape: {}'.format(self.shape))
        self.print('max_energy: {}'.format(self.max_energy))
        self.print('coin_box (x,y): {}'.format(self.coin_box))
        self.print('')


    def step(self, observation):
        ''' Called every time an observation has been received

            Args:
                observations (strategy.Observation):
                    Container with all robots' observations
            
            Returns:
                actions (strategy.Action):
                    Container with all robots actions 
        ''' 

        # Observation
        self.print('score: {}'.format(observation.score))
        self.print('opponent_score: {}'.format(observation.opponent_score))
        self.print('added_coins: {}'.format(observation.added_coins))
       
        for robot_id in range(self.num_robots):
            robot = observation.robot(robot_id)
            self.print('Observation (robot {})'.format(     robot_id                                    ))
            self.print('energy: {}'.format(                 robot.energy                                ))
            self.print('penalty: {}'.format(                robot.penalty                               ))
            self.print('has_item: {}'.format(               robot.has_item                              ))
            self.print('home_base: {}'.format(              robot.home_base                             ))
            self.print('position(x,y): ({},{})'.format(     *robot.position                             ))
            self.print('left: {} {}'.format(                robot.left.distance, robot.left.object      ))
            self.print('right: {} {}'.format(               robot.right.distance, robot.right.object    ))
            self.print('top: {} {}'.format(                 robot.up.distance, robot.up.object          ))
            self.print('bottom: {} {}'.format(              robot.down.distance, robot.down.object      ))
            self.print('')

        # Action
        action = self.action()
        for robot_id in range(self.num_robots):
            self.print('Action (robot {})'.format(  robot_id                                ))
            self.print('detect: {}'.format(         action.detect(robot_id)                 ))

            self.print('none: {}'.format(           action.none(robot_id)                   ))
            self.print('move: {}'.format(           action.move(robot_id, 'left')           ))
            self.print('fake coin: {}'.format(      action.fake_coin(robot_id, 'up')        ))
            self.print('share_energy: {}'.format(   action.share_energy(robot_id, 'down')   ))
            self.print('recharge {}'.format(        action.recharge(robot_id) ))
            self.print('move: {}'.format(           action.move(robot_id, 'right')          ))
            
            self.print('move: {}'.format(           action.move(robot_id, 'aaaa')           ))
            self.print('fake coin: {}'.format(      action.fake_coin(robot_id, 'bbbb')      ))
            self.print('share_energy: {}'.format(   action.share_energy(robot_id, 'cccc')   ))
            self.print('move: {}'.format(           action.detect(-1) ))
            self.print('')

        self.print(action.get())

        return action


# Run strategy
if __name__ == "__main__":
    test = Test()
    test.run()
