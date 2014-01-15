#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "nanopad2twist")

(defun joy2twist
  (msg)
  (let* ((x (* 2 (- (aref (send msg :axes) 1) 0.5)))
	 (y (* 2 (- (aref (send msg :axes) 0) 0.5)))
	 (r (atan2 y x))
	 (twist (instance geometry_msgs::twist :init)))
    (cond
     ((or (> (abs x) 1e-3)
	  (> (abs y) 1e-3))
      (send (send twist :linear) :x x)
      (send (send twist :linear) :y y)
      (send (send twist :linear) :z 0)
      (send (send twist :angular) :x 0)
      (send (send twist :angular) :y 0)
      (send (send twist :angular) :z (* -1.0 r))
      (ros::publish "/sphero_node/cmd_vel"
		    twist)))
    ))

(ros::advertise "/sphero_node/cmd_vel"
                geometry_msgs::twist)
(ros::subscribe "/nanopad2/joy"
		sensor_msgs::Joy
                #'joy2twist
		1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep))
