# Comprobo Warmup Project

*ENGR3590: A Computational Introduction to Robotics, Olin College of Engineering, FA2022*

*Simrun Mutha and Melody Chiu*

## Introduction

The objective of this warmup project is to gain familiarity with ROS and brush up on Python through programming the following robot behaviors using data from the laser range finder and bump sensors:

* [Teleoperation](#teleoperation)
* [Driving a Square](#square-driver)
* [Wall Following](#wall-following)
* [Person Following](#person-following)
* [Obstacle Avoidance](#obstacle-avoidance)

Through working on this project, we've learned strategies for debugging robotics programs and gained familiarity with finite-state robot control. We ran our programs on the [Neato](https://neatorobotics.com/) robot vacuum, both physically and in simulation through [Gazebo](https://gazebosim.org/home).

## Teleoperation

[[source]](warmup/warmup/teleop.py)

## Driving a Square

[[source]](warmup/warmup/drive_square.py)

## Wall Following

[[source]](warmup/warmup/wall_follower.py)

<img src="warmup/resource/wall_follower.gif" width="280"/>

The goal for this behavior is to have the Neato move forward while aligning its direction of motion to be parallel to the nearest wall. Our implementation involves using data from the laser range finder to determine whether the robot should a) keep driving straight, b) steer slightly to the left or c) steer slightly to the right in order to keep parralel to the wall. 

## Person Following

[[source]](warmup/warmup/person_follower.py)

## Obstacle Avoidance

[[source]](warmup/warmup/obstacle_avoidance.py)