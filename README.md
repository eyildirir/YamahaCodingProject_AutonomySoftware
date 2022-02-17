# Yamaha Coding Project - Autonomy Software
## Brief
This is a simple boat simulator with a custom PID controller implementation for reaching and holding a goal speed. It can accomodate for up to 6 engines on a boat. It utilizes a second degree backwards difference algorithm to implement a PID regulator for controlling the change in throttle, which ultimately acts as a PID controller for the speed of the boat itself.

## Compiling
There is a simple Shell Script provided that, when run in the project root directory, compiles executables for the main simulator as well as a few unit tests for testing PID Controller functionality.

The simulator and the test executables can also be manually compiled by running:
```
# Main Simulator Program:
g++-9 -std=c++2a src/main.cpp -o simulator.o -lboost_filesystem -lboost_iostreams

# Tuning Tester Program for testing coefficients:
g++-9 -std=c++2a test/tuning_test.cpp -o simulator.o -lboost_filesystem -lboost_iostreams

# Unit Tests for PIDController:
g++-9 -std=c++2a test/pidControllerTests.cpp -o unittest.o
```

## Running the Simulator
The simulator can be run by callling `./simulator.o n_engines` where `n_engines` is the number of engines on the boat (1-6).

The tuning test can be run by calling `./tuning_test.o kp kd ki (desired_speed) (initial_speed) (max_sim_time) (time_step)` where the inputs in parantheses are incrementally optional.

The unit tests can be run simply by calling `./unittest.o`.