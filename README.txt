Robot explorers simulator.

Copyright (c) 2020 Huawei Technologies Sweden AB, All rights reserved.
By running, modifying or otherwise using the source code and resources in this project 
you accept and agree with the license. See "LICENSE.txt" for further details.

Authors:
  Karl Gäfvert
  Romain Deffayet

Please notify a mentor if you're having issues with setting up or running the simulator.
If you encounter a bug please contact Karl through Slack or email karl.gafvert2@huawei.com 
as soon as possible.

The default essentials environment on the submission platform has the following libraries 
installed:
 - Numpy
 - Scipy
 - Scikit learn
 - igraph
 - PyTorch (CPU only)
 - Pyro
 - CVXPY

If you have any requests for additional libraries please contact a mentor and explain why
you think it is essential.

We have verified the simulator for Python 3.7
  - Windows
  - Linux (Ubuntu 18.04)
  - MacOS

If the simulator is working for you with a different OS and or Python version please let 
us know so we can update this list.

--- Installation ---
  1. Setup your favorite virtual environment
  2. pip install numpy videofig
  3. Unzip "simulator-vxx.zip"
  4. Open a terminal
  5. Enter the "simulator" folder

--- Implementing your own stragey ---
  Each strategy is a self-contained program launched by the simulator as a subprocess.
  The simulator and each subprocess communicates through stdin and stdout by sending
  utf-8 encoded JSON-strings with the observations and actions. stderr is also captured
  as debug data. This makes it possible to interface any programming language with the 
  simulator. We provide a reference Python implemenation of the communication protocol, 
  if you are interested in using another programming language for your strategy, please 
  contact a mentor as soon as possible.

  Template for strategies:

    from robot_explorers import Strategy
    
    class MyStrategy(Strategy):        
        def __init__(self):
            super().__init__()

        def init(self):
            # init(...) runs once after initial game info have been received from the simulator
        
        def step(self, observation):
            # step(...) is called every time a new observation is received from the simulator
            
            # debug print energy of robot 0
            self.print('Energy: ', observations.robot(0).energy)

            # Get empty action
            action = self.action()

            # Update action with move for robot 1
            action.move(1, 'right')

            return action

        # Run strategy
        if __name__ == "__main__":
            my_strategy = MyStrategy()
            my_strategy.run()

  You can find two example implementations "test_strategy.py" and "random_strategy.py" 
  distributed with the simulator.

--- Working with the simulator ---
  The simulator is launched from a terminal.
  You can modify all important parameters of the simulator directly in the files 
  "robot_explorers/run.py" and "robot_explorers/simulator.py".
  We will provide the set of parameters used for final evaluation when the online
  submission platform opens.

  TLDR

    When developing:
      $ python -m robot_explorers my_strategy.py random_strategy.py --timesteps 10 --debug-strategy --silent-strategy-1

    When testing/tweaking parameters:
      $ python -m robot_explorers my_strategy.py random_strategy.py --timesteps 200 --fps 10 --silent-strategy-1
  
  END TLDR

  You can launch the simulator with the command:
    $ python -m robot_explorers strategy0 strategy1

  The two mandatory arguments "strategy0" and "strategy1" should be the file names of 
  two strategies you want to run in the simulator.
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py

  By default the simulator only runs 1 time step. To run more timesteps use 
  the "--timesteps N" flag. 
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py --timesteps 100

  When developing your own strategy, it is possible to capture the output between the 
  simulator and your strategy with the "--debug-strategy" flag.
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py --timesteps 10 --debug-strategy

  Visualize the simulation and control the FPS with the "--visualize" and "--fps N" flags.
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py --timesteps 100 --debug-strategy --visualize --fps 5

  It is also possible to save the visualizations, observations, actions, and debug 
  output of your strategies. The flags "--headless" and "--output-dir OUTPUT_DIR" 
  enables saving to disk and specifies the location where the folder with the new 
  results is created, it defaults to OUTPUT_DIR="results".
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py --timesteps 100 --headless --fps 5

  The flags "--silent-strategy-0" and "--silent-strategy-1" disables most 
  output for their respective strategies. They are used in conjunction with
  "--debug-strategy" and or "--visualize".
  
  Example:
    $ python -m robot_explorers test_strategy.py random_strategy.py --timesteps 100 --debug-strategy --silent-strategy-0

  See help for some additional information:
    $ python -m robot_explorers random_strategy.py --help

  Output:
    usage: __main__.py [-h] [--timesteps TIMESTEPS] [--visualize] [--fps FPS]
                    [--headless] [--output-dir OUTPUT_DIR]
                    [--silent-strategy-0] [--silent-strategy-1]
                    [--debug-strategy] [--debug-sim] [--about]
                    strategy0 strategy1

    Run simulator

    positional arguments:
    strategy0             Path to strategy file. Ex: "./basic_strategy.py"
    strategy1             Path to strategy file. Ex: "./my_strategy.py"

    optional arguments:
    -h, --help            show this help message and exit
    --timesteps TIMESTEPS
                            Number of timesteps to run
    --visualize           Visualize simulation in display window
    --fps FPS             FPS during visualization
    --headless            Save simulation visualizations and debug info to disk
    --output-dir OUTPUT_DIR
                            Output directory. Ex. "./output"
    --silent-strategy-0   Disable strategy 0 simulator output
    --silent-strategy-1   Disable strategy 1 simulator output
    --debug-strategy      Print debug information for strategies
    --debug-sim           Print debug information for simulator
    --about               Print info and license

  Visualization of saved and downloaded data: 
    $ python -m robot_explorers.gui INPUT_DIR

  The API is simple and follows the main simulator:
    $ python -m robot_explorers.gui --help
  
  Output:
    usage: __main__.py [-h] [--fps FPS] [--silent-strategy-0]
                    [--silent-strategy-1] [--about]
                    input_dir

    Run GUI

    positional arguments:
    input_dir            Path to saved simulation. Ex. "results/10212020_021804"

    optional arguments:
    -h, --help           show this help message and exit
    --fps FPS            FPS during visualization
    --silent-strategy-0  Disable strategy 0 simulator output
    --silent-strategy-1  Disable strategy 1 simulator output
    --about              Print info and license

--- Submissions ---
  The number of available submissions/day is going to be announced when the submission 
  platform opens. Simulator parameters for the submission platform are going to be 
  announced when the submission platform opens.

  See list of available libraries.
  You can request additional libraries to be added to the common environment. 
  Any environment updates will be shared with all other teams (the mentors reserve 
  the right to deny any library request for any reason). Even if a request to add a  
  library to the public environment gets denied, you are still allowed to use any 
  library you want as long as it is included in your upload and loaded by your code. 
  Please note that you are responsible for making sure that any libraries or code 
  that you include in your upload must be compatible with the simulator license 
  (see LICENSE.txt).

  1. Rename your main-file to "solution.py"
  2. Make sure all your code loads through "solution.py"
  3. Put your code in a zip archive
  4. Go to https://hackathonse.huawei.com (opens November 2) and upload the zip-archive
  5. The ranking list will be updated when the system has finished running your solution
