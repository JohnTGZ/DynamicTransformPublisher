# camera_calibration
Calibrate the position and orientation of a camera using fiducial markers.

This is done by obtaining the transformation of the camera relative to the fiduciary markers. And then we fix the transformation of base_link relative to the fiduciary marker.

The current implementation uses apriltags.

The transformation tree for an example camera is as follows:

<img src="imgs/tf_tree.png" alt="tf_tree" width="525"/>

# Dependencies
1. [ddynamic_reconfigure]

# Quick Start


# Dynamic TF Broadcaster
This node basically enables you to adjust the TF during runtime (rather than restart it over and over again).
There are 4 rotation modes you can toggle between
1. Quaternion
2. XYZ Euler Rotation
3. ZYX Euler Rotation
4. RPY Rotation 
   1. Absolute rotation (the axis itself does not change orientation with each rotation)

# Additions Planned

1. Change the trans and quat variables to vector for brevity
2. Add option to toggle between different Euler angle orders
3. Add axis angle 
4. Add rotation matrix 
5. Add axis with angle magnitude 
6. Add web UI

# Issues

