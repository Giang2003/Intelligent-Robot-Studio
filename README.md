# Intelligent-Robot-Studio

The Arduino Robot Project, a versatile set of Arduino code and hardware configurations for creating autonomous, line-following, obstacle-avoidance, object detection, and path-planning robots. This comprehensive project includes multiple code files, each tailored to specific robot functionalities.
## Table of Contents

- [Introduction](#introduction)
- [Components and Materials](#components-and-materials)
- [Code Files](#code-files)
- [Getting Started](#getting-started)

## Introduction

This project is a collection of Arduino code and hardware configurations for building various types of robots, including:

- **Autonomous Robot**: An autonomous robot capable of moving forward, backward, left, and right. It's ideal for testing motors and creating the foundation for autonomous navigation.
  
- **Line-Following Robot**: A robot designed to follow a white line on the ground. It uses sensors to stay on the line and make turns as needed.
  
- **Obstacle Avoidance Robot (`obstacle.ino`)**: An obstacle-avoidance robot equipped with ultrasonic sensors. It can autonomously navigate its environment, detect obstacles, and choose safe paths to avoid collisions.
  
- **Bottle Detection with Raspberry Pi**: A Raspberry Pi-based system detects green bottles using OpenCV for color segmentation. It uses a PID controller to align with the object and communicates with Arduino for movement and gripping actions.

- **Path-Planning with Lidar**: Using Lidar for real-time environment mapping, the robot can plan safe paths, avoiding obstacles and dynamically adjusting its route with algorithms like A*.

## Components and Materials

Before starting this project, you will need the following components and materials based on the specific robot you want to build:

- Arduino board (Arduino Uno)
- Motor driver (e.g., L298N) for motor control
- Ultrasonic sensors (HC-SR04) for obstacle detection
- Line tracking sensors 
- BNO055 sensor
- Lidar LD19
- Logitech camera
- Embedded Computer Raspberry Pi 
- Motors, power sources specific to robot design
- Jumper wires for connecting components
- A breadboard (optional)

## Code Files

This project includes several code files, each tailored to a specific type of robot:

- `avoid_obstacle.ino`: This code employs IR sensors for following a white line on the ground and ultrasonic sensors to autonomously navigate, detect obstacles, and choose safe paths.

- `grab_bottle.ino`: This file is designed for the robot to detect and grab a bottle based on a certain distance and alignment using sensors, then trigger a gripping mechanism upon detection.

- `lidar_collect.ino`: This code is used to integrate Lidar sensors, collecting data to assist in navigation and environment mapping for autonomous path planning.

- `detect_bottle.py`: A Python script used with the Raspberry Pi for bottle detection using image processing with OpenCV, capturing live frames to identify and track objects.

- `lidar_collector.py`: A Python script for collecting and processing data from Lidar sensors, used to create detailed environmental maps for the robot’s path planning.

- `astarpath.py`: A Python script that implements the A* algorithm for finding the shortest and safest path based on Lidar data.

## Getting Started

To get started with your robot project, follow these general steps:

1. Assemble the hardware components based on the specific robot design.
2. Upload the corresponding code to your Arduino board using the Arduino IDE.
3. Connect to Raspberry Pi, run Python files from terminal.
4. Power up the robot and observe its behavior.
