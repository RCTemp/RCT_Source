#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "ROS2OSC")

(ros::roseus "ros2osc_pub_sub")

(defun str_echo (msg)
  (print (send msg :data)))

(defun ros2osc_pub
  (&key
   (ip "127.0.0.1")
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

(ros::subscribe "/ros2osc/feedback"
		std_msgs::string
                #'str_echo
		1)

(ros::advertise "ros2osc/request"
                ros2osc::ros2osc)

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros2osc_pub :address "/test1")
 (ros::spin-once)
 (ros::sleep))
