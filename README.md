# Easing the Effects of Delays by Introducing Idle Behaviors

## Installation

Clone this repo into a catkin_ws that also has the packages necessary
to run the BWI ArmBot.

## Usage

    roslaunch tic_tac_toe play_game.launch

## Development

### System Layout

* Interaction Manager: Orchestrates the game interaction. This includes start and end behavior, as well as keeping track of turn taking, and deciding on actions

* Action Executor: Controls the motion of the arm

* Game State Detector: Extracts the game state from the robot's vision
