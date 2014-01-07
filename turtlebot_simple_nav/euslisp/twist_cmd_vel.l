#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "twist_cmd_vel")

(defvar *now-velocity*
  (instance geometry_msgs::twist :init))
(defvar *time-step* 0.1)
(defun acc2vel
  (acc)
  (mapcar
   #'(lambda (v)
       (mapcar
	#'(lambda (k)
	    (send (send *now-velocity* v) k
		  ;;(+ (send (send *now-velocity* v) k)
		  (* 1e-1
		     (send (send acc v) k))))
	'(:x :y :z)))
   '(:linear :angular)))

(defun scale-vel
  (scale)
  (mapcar
   #'(lambda (v)
       (mapcar
	#'(lambda (k)
	    (send (send *now-velocity* v) k
		  (* scale (send (send *now-velocity* v) k))))
	'(:x :y :z)))
   '(:linear :angular)))

(ros::advertise "/cmd_vel_mux/input/teleop"
                geometry_msgs::twist)
(ros::subscribe "/turtlebot_controller/twist_command"
		geometry_msgs::twist
                #'acc2vel
		1)

(ros::rate (round (/ 1.0 *time-step*)))
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::publish "/cmd_vel_mux/input/teleop" *now-velocity*)
 (scale-vel 0.8)
 (ros::sleep))
