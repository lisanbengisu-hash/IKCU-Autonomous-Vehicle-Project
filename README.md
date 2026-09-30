# IKCU Autonomous Vehicle Project

Autonomous campus shuttle simulation developed for Izmir Katip Celebi University using CARLA, Unreal Engine 5, Python and computer vision.

## Project Overview

The project simulates an autonomous shuttle operating on a custom digital model of the IKCU campus.

The system includes:

- Custom IKCU campus map
- OpenDRIVE road network
- Multi-stop route planning
- Autonomous vehicle control
- Pure Pursuit steering control
- PID-based speed control
- Pedestrian detection and safety logic
- Vehicle detection and adaptive following
- YOLO-based computer vision
- NPC vehicle and pedestrian scenarios
- Collision avoidance and route bypass logic
- Custom bus stop system

## Technologies

- CARLA 0.10.0
- Unreal Engine 5.5
- Python 3.8
- OpenDRIVE
- YOLO
- Git
- Git LFS

## Repository Structure

### Python

Contains the autonomous driving and simulation software.

Important modules include:

- `main_v8_multistop_v4_curve_safe.py` - Main simulation and system integration
- `route_planner.py` - Route planning
- `vehicle_controller.py` - Vehicle control
- `vision_system.py` - Computer vision system
- `yolo_detector.py` - YOLO detection
- `route_bypass.py` - Route bypass and collision handling
- `special_routes.py` - Special route logic
- `spawn_manager.py` - Vehicle spawning
- `bus_stops.py` - Campus bus stop definitions
- `ui.py` - Route/stop selection interface

### OpenDrive

Contains the OpenDRIVE road network used by the custom IKCU campus map.

### Unreal

Contains the custom IKCU campus Unreal Engine map and related assets.

Large Unreal Engine binary files (`.umap` and `.uasset`) are managed using Git LFS.

## Current Status

The custom IKCU campus map has been successfully packaged as a Windows standalone CARLA build.

The Python autonomous driving system has been connected to the standalone CARLA server and tested on the custom campus map.

Current development focuses on improving route following, turning behavior, vehicle control and continuous campus shuttle operation.

## Development Workflow

Source-code and map development are version controlled with Git and GitHub.

Python changes can be committed frequently after functional improvements.

Because Unreal Engine map files are large binary assets, map changes are committed after meaningful map-development milestones.

## Note

The packaged Windows CARLA build is distributed separately from this repository.

Build outputs, generated files, caches and temporary Unreal Engine files are intentionally excluded from Git version control.
