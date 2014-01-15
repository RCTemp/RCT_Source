#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "map_converter_test")

(defun converted_xyy_echo
  (msg)
  (print (send msg :data)))

(ros::subscribe "/map_converter/converted_xyy"
		std_msgs::float32multiarray
		#'converted_xyy_echo 1)
(ros::advertise "/map_converter/xyy"
		std_msgs::float32multiarray 1)
(ros::advertise "/map_converter/posestamped"
		geometry_msgs::posestamped 1)

(ros::publish
 "/map_converter/xyy"
 (instance std_msgs::float32multiarray :init
	   :data #f(133 141 0)))
(unix:sleep 1)
(ros::spin-once)
