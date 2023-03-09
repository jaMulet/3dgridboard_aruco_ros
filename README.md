3d grid aruco for ros
=========

C++ package and ROS wrappers of the [3d gridboard aruco][1] detector. This repository is based on the [aruco_ros][2] library.

# ROS API

## Topics

 * /camera_info

 Inputs the camera cjharacteristics and features.

        Type: sensor_msgs/CameraInfo

 * /image

 Inputs the image to be evaluated.

        Type: sensor_msgs/Image

 * /aruco_3dgrid/result

 Outputs the evaluated image.

        Type: sensor_msgs/Image

 * /aruco_3dgrid/pose

 Ouputs position and orientation of the central marker (raw detection).
 
        Type: geometry_msgs/PoseStamped

 * /aruco_3dgrid/position

 Outputs position of the central marker (raw detection).

        Type: geometry_msgs/Vector3Stamped

 * /aruco_3dgrid/transform

 Outputs transformation of the central marker (raw detection).

        Type: geometry_msgs/TransformStamped

 * /aruco_3dgrid/pose_grid

 Outputs position and orientation of the grid.

        Type: geometry_msgs/PoseStamped

 * /aruco_3dgrid/position_grid

 Outputs position of the grid.

        Type: geometry_msgs/Vector3Stamped

* /aruco_3dgrid/transform_grid

 Outputs transformation of the grid.

        Type: geometry_msgs/TransformStamped

 * /aruco_3dgrid/transforms

 Outputs an array of transformations of all detected markers.

        Type: aruco_msgs/TransformArray

## Messages

 * aruco_ros/Marker.msg

        Header header
        uint32 id
        geometry_msgs/PoseWithCovariance pose
        float64 confidence

 * aruco_ros/MarkerArray.msg

        Header header
        aruco_ros/Marker[] markers

 * aruco_ros/MarkerArray.msg

        Header header
        geometry_msgs/TransformStamped[] transforms

# Ackownledge

This work has been developed in the context of the H2020-MSCA-ITN DiManD project funded by the European Union’s Horizon
2020 research and innovation programme under grant agreement no.814078.

<img align="right" src="https://raw.github.com/jaMulet/3dgridboard_aruco_ros/master/3dgridboard_aruco_ros/images/dimand_MSCA-ITN.png"/>


[1]: https://doi.org/10.3390/s20174825 "Oščádal P, Heczko D, Vysocký A, Mlotek J, Novák P, Virgala I, Sukop M, Bobovský Z. Improved Pose Estimation of Aruco Tags Using a Novel 3D Placement Strategy. Sensors. 2020; 20(17):4825"

[2]: http://wiki.ros.org/aruco_ros "Aruco_ros"