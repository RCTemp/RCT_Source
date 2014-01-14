#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "joy2twist")

(defun joy2twist
  (msg)
  (let* ((twist (instance geometry_msgs::twist :init)))
    (send (send twist :linear) :x
	  (* -1 (aref (send msg :axes) 1)))
    (send (send twist :linear) :y 0)
    (send (send twist :linear) :z 0)
    (send (send twist :angular) :x 0)
    (send (send twist :angular) :y 0)
    (send (send twist :angular) :z
	  (* -1 (aref (send msg :axes) 3)))
    (ros::publish "/turtlebot_controller/twist_command"
		  twist))
  )

(ros::advertise "/turtlebot_controller/twist_command"
                geometry_msgs::twist)
(ros::subscribe "/simple_joy/joy"
		sensor_msgs::Joy
                #'joy2twist
		1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep))
