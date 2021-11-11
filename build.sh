#!/bin/bash

gcc `pkg-config --cflags k4a` -I/usr/include -o get_2d get_2d_skeleton.c matrix.c `pkg-config --libs k4a` -L/usr/lib -lk4abt -lm

