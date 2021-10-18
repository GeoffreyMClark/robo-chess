# Robo-Chess

This entire library is being actively worked on and is in no way complete.

Robo-chess is a Python library for playing chess in real-time. While other chess playing robots have been around for awhile, they tend to put environmental limitations on the board, pieces and lighting. This library is a step toward a generneralized robotic framework for chess. The library consists of three main sections which I have endeavored to make easy to mixed and match with your own custom code. 

The library is broken down into three main sections: vision -physical chess board and pieces are decomposed into a virtual board state and localided in the world-, strategy -optimal move is decided-, and motor primitives -robot trajectory is composed and executed-. As you can see the board state is not updated solely from the robots perception of the board therefore mistakes or miss-moves must be check for and fixed using the same process.

## Vision
The vision subsection has three main components:
1. Percieve the physical board and pieces and render a board state.
2. Check to ensure all moves made are valid.
3. Localize the board and pieces in reference to the robot.

## Strategy
1. If an invalid move was made plan a move correction.
2. Use Deepminds AlphaZero to construct optimal move from the board state.

## Motor Primitives
1. Plan movement trajectory using neural movement primitives conditioned on piece current and goal locations.