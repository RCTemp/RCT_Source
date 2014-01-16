#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")

(defun sound-req-echo
  (msg)
  (unix:system "aplay ../raw/bomb.mp3")
  )

(ros::subscribe
 "/topic"
 std_msgs::empty
 #'sound-req-echo 1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
