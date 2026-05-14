"""
Horizon Mapper Node
===================

Version: 1.0.0
Author: Mohammed S. Azab Abdelazim (Mohammed@azab.io)

Description:
------------
ROS2 node for trajectory mapping and predictive horizon generation for model-based controllers.
Provides adaptive bounds, visualization, and runtime adjustments.

Features:
- Loads and preprocesses trajectory data
- Publishes reference and constrained trajectories
- Provides visualization capabilities
- Handles runtime bound adjustments
- Validates trajectory and vehicle state

Interfaces:
- nav_msgs.msg.Path, Odometry
- geometry_msgs.msg.PoseStamped, PoseWithCovarianceStamped, Point
- std_msgs.msg.Bool, Header, ColorRGBA
- visualization_msgs.msg.Marker, MarkerArray
- giu_f1t_interfaces.msg.VehicleState, VehicleStateArray, ConstrainedVehicleStateArray, ConstrainedVehicleState, BoundAdjustment, ConstraintStatus

Copyright (c) 2025 GIU-F1Tenth
License: MIT
"""

""" 
Sanity Check:
Run the following command to verify that the trajectory preprocessing script works correctly.
Replace /path/to/trajectory.csv with the actual path to your trajectory file.
    
    python3 preprocess_trajectory.py --sanity-check /path/to/trajectory.csv
   
   # or if you want to use the default trajectory file, simply run:
    python3 preprocess_trajectory.py --sanity-check

"""