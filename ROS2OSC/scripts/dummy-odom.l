#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "dummy_odom")

(ros::advertise "/odom"
                geometry_msgs::posestamped)

(ros::rate 10)
(let ((odom (instance geometry_msgs::posestamped :init)))
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (print 'running)
   (send (send (send odom :pose) :position) :x (random 1.0))
   (send (send (send odom :pose) :position) :y (random 1.0))
   (ros::publish "/odom" odom)
   (ros::spin-once)
   (ros::sleep)))
