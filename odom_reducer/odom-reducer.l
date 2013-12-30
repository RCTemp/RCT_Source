#!/usr/bin/env roseus

(defvar *odom-reducer-cnt* 0)
(defun pose-callback
  (msg)
  (let ((pos (send (send (send msg :pose) :pose) :position)))
    (cond
     ((evenp *odom-reducer-cnt*)
      (ros::publish "/ros2osc/float/ballx"
		    (instance std_msgs::float32 :init :data (send pos :x))))
     (t
      (ros::publish "/ros2osc/float/bally"
		    (instance std_msgs::float32 :init :data (send pos :y))))
     )))

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "odom_reducer")

(ros::advertise "/ros2osc/float/ballx" std_msgs::float32)
(ros::advertise "/ros2osc/float/bally" std_msgs::float32)

(ros::subscribe
 "/odom"
 nav_msgs::Odometry
 #'pose-callback 1)

(ros::rate 5)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 (setq *odom-reducer-cnt*
       (mod (+ *odom-reducer-cnt* 1) 2))
 )
