#!/bin/bash

g++ src/test.cpp src/Constraints.cpp src/Path.cpp src/Trajectory.cpp -I./include -std=c++20 -O3 -ffast-math -DNDEBUG -march=native -o test