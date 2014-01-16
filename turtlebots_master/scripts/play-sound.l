#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "play_sound")

(defun sound-req-echo
  (msg)
  (unix:system "aplay ../music_raw/bomb.mp3")
  )

(defun sound-req-echo2
  (msg)
  (unix:system "aplay ../music_raw/roar_zombie2.mp3")
  (unix:system "aplay ../music_raw/roar_zombie.mp3")
  )

(ros::subscribe
 "/dead"
 std_msgs::empty
 #'sound-req-echo 1)

(ros::subscribe
 "/clear"
 std_msgs::empty
 #'sound-req-echo2 1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 ;;(print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
