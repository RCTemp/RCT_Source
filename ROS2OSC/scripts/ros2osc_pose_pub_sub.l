#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "ROS2OSC")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "ros2osc_pose")

(defvar *ros2osc-pose-topic-name*
  (or (unix:getenv "ROS2OSC_POSE_TOPIC_NAME") "/odom"))
(defvar *ros2osc-pose-topic-class*
  (eval
   (read-from-string
    (or (unix:getenv "ROS2OSC_POSE_TOPIC_CLASS")
	"geometry_msgs::posestamped"))))
(defvar *ros2osc-pose-hz*
  (read-from-string
   (or (unix:getenv "ROS2OSC_POSE_HZ") "10")))
(defvar *osc-host-ip*
  (or (unix:getenv "OSC_HOST_IP") "127.0.0.1"))

(defun str_echo (msg)
  (print (send msg :data)))

(defun ros2osc_pub
  (&key
   (ip *osc-host-ip*)
   (port "8000")
   (address "/test")
   (message (list "string" 1 10.0)))
  (let* ((msg (instance ros2osc::ros2osc :init))
	 type osc)
    (send msg :osc_ip ip)
    (send msg :osc_port port)
    (send msg :osc_address address)
    (send msg
	  :osc_messages
	  (mapcar
	   #'(lambda (buf)
	       (setq osc (instance ros2osc::osc :init))
	       (cond
		((stringp buf)
		 (send osc :type "string")
		 (send osc :str buf))
		((floatp buf)
		 (send osc :type "float")
		 (send osc :float buf))
		((integerp buf)
		 (send osc :type "integer")
		 (send osc :int buf)))
	       osc)
	   message))
    (ros::publish "ros2osc/request" msg)))

(defvar *now-pose*)
(defun pose-stamped-echo
  (msg)
  (cond
   ((subclassp (class msg) geometry_msgs::posestamped)
    (setq *now-pose* (send msg :pose)))
   ((subclassp (class msg) geometry_msgs::pose)
    (setq *now-pose* msg))))

;;(ros::subscribe "/ros2osc/feedback"
;;		std_msgs::string
;;                #'str_echo
;;		1)
(ros::advertise "ros2osc/request"
                ros2osc::ros2osc)
(ros::subscribe *ros2osc-pose-topic-name*
		*ros2osc-pose-topic-class*
		;;geometry_msgs::posestamped
                #'pose-stamped-echo
		1)

(ros::rate (* 2 *ros2osc-pose-hz*))
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (setq *now-pose* nil)
 (ros::spin-once)
 (ros::sleep)
 (if *now-pose*
     (ros2osc_pub
      :ip *osc-host-ip*
      :address "/ballx"
      :message (list (send (send *now-pose* :position) :x))))
 (ros::sleep)
 (if *now-pose*
     (ros2osc_pub
      :ip *osc-host-ip*
      :address "/bally"
      :message (list (send (send *now-pose* :position) :y)))))
