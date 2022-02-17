#!/bin/sh

# Main Simulator Program:
g++-9 -std=c++2a src/main.cpp -o simulator.o -lboost_filesystem -lboost_iostreams

# Tuning Tester Program for testing coefficients:
g++-9 -std=c++2a test/tuning_test.cpp -o simulator.o -lboost_filesystem -lboost_iostreams

# Unit Tests for PIDController:
g++-9 -std=c++2a test/pidControllerTests.cpp -o unittest.o