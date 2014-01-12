#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "ROS2OSC")

(defvar *node-name*
  (or (unix:getenv "MAPCONV_NODE_NAME")
      "map_converter"))
(defvar *rosrun-hz* 10)

(defvar *converted-xyy-topic-name*
  (or (unix:getenv "MAPCONV_CONVERTED_TOPIC_NAME")
      (format nil "/~A/converted_xyy" *node-name*)))
(defvar *xyy-req-topic-name*
  (or (unix:getenv "MAPCONV_FLOATARRAY_REQ_TOPIC_NAME")
      (format nil "/~A/xyy" *node-name*)))
(defvar *pose-req-topic-name*
  (or (unix:getenv "MAPCONV_POSE_REQ_TOPIC_NAME")
      (format nil "/~A/posestamped" *node-name*)))
(defvar *osc-host-ip*
  (or (unix:getenv "OSC_HOST_IP") "127.0.0.1"))
(defvar *turtlebot-id*
  (or (unix:getenv "TURTLEBOT_ID") "controll"))

(ros::roseus *node-name*)

(defvar *sampling-points*
  (list
   (list #F(1.37 1.41) #F(1424.4 1322.2))
   (list #F(1.39 -1.48) #F(1435 1322.2))
   (list #F(-0.66 -1.43) #F(1435 1313))
   (list #F(-0.71 1.20) #F(1424.4 1313))))

(defvar *R*)
(defvar *b*)

(let* ((y
	(apply
	 #'concatenate
	 (cons float-vector
	       (mapcar #'cadr *sampling-points*))))
       (mat (make-matrix (* 2 (length *sampling-points*)) 6))
       (index 0)
       (x
	(progn
	  (dolist (l (mapcar #'car *sampling-points*))
	    (setf (aref mat index 0) (aref l 0))
	    (setf (aref mat index 1) (aref l 1))
	    (setf (aref mat index 4) 1)
	    (incf index)
	    (setf (aref mat index 2) (aref l 0))
	    (setf (aref mat index 3) (aref l 1))
	    (setf (aref mat index 5) 1)
	    (incf index))
	  mat))
       (param (transform (pseudo-inverse x) y)))
  (setq *b* (subseq param 4 6))
  (setq *R* (make-matrix 2 2))
  (setf (aref *R* 0 0) (aref param 0))
  (setf (aref *R* 0 1) (aref param 1))
  (setf (aref *R* 1 0) (aref param 2))
  (setf (aref *R* 1 1) (aref param 3)))

(defun transform-pos
  (pos) (v+ *b* (transform *R* pos)))

(defun transform-rot
  (deg)
  (let* ((x #F(1 0))
	 (y (normalize-vector (transform *R* x)))
	 (axis
	  (aref
	   (v*
	    (concatenate float-vector x #F(0))
	    (concatenate float-vector y #F(0)))
	   2))
	 (rot (* (if (> axis 0) 1 -1) (acos (v. x y)))))
    (+ deg (rad2deg rot))))

(defun xyy-echo
  (msg)
  (let (pos rot)
    (setq pos
	  (transform-pos (subseq (send msg :data) 0 2)))
    (setq rot
	  (transform-rot (aref (send msg :data) 2)))
    (format t "~A -> ~A~%"
	    (concatenate float-vector
			 (subseq (send msg :data) 0 2)
			 (float-vector (aref (send msg :data) 2)))
	    (concatenate float-vector
			    pos (float-vector rot)))
    (ros2osc_pub
     :address "/position"
     :message
     (list *turtlebot-id*
	   (aref pos 0) (aref pos 1) rot))
    ))

(defun pose-echo
  (msg)
  (let (pose pos rot)
    (cond
     ((subclassp (class msg) geometry_msgs::posestamped)
      (setq pose (send msg :pose)))
     ((subclassp (class msg) geometry_msgs::pose)
      (setq pose msg))
     ((subclassp (class msg) nav_msgs::odometry)
      (setq pose (send (send msg :pose) :pose))))
    (xyy-echo
     (instance
      std_msgs::float32multiarray
      :init
      :data
      (concatenate
       float-vector
       (float-vector
	(send (send pose :position) :x)
	(send (send pose :position) :y))
       (float-vector
	(rad2deg
	 (aref
	  (matrix-log
	   (quaternion2matrix
	    (map float-vector
		 #'(lambda (k)
		     (send (send pose :orientation) k))
		 '(:w :x :y :z))))
	  2))))))))

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
    ;;(describe msg)
    (ros::publish "ros2osc/request" msg)))

(ros::advertise "ros2osc/request"
                ros2osc::ros2osc)
;; (ros::subscribe "/mobile_base/events/bumper"
;; 		kobuki_msgs::BumperEvent
;; 		#'bumper-echo
;; 		1)

(ros::advertise
 *converted-xyy-topic-name*
 std_msgs::float32multiarray 1)
(ros::subscribe
 *xyy-req-topic-name*
 std_msgs::float32multiarray
 #'xyy-echo 1)
(ros::subscribe
 *pose-req-topic-name*
 geometry_msgs::posestamped
 #'pose-echo 1)

(ros::rate *rosrun-hz*)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )


#|

#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "ROS2OSC")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "ros2osc_pose")

(defvar *ros2osc-pose-topic-name*
  (or (unix:getenv "ROS2OSC_POSE_TOPIC_NAME") "/odom"))
(defvar *ros2osc-pose-topic-class*
  (eval
   (read-from-string
    (or (unix:getenv "ROS2OSC_POSE_TOPIC_CLASS")
	"nav_msgs::odometry"))))
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
    ;;(describe msg)
    (ros::publish "ros2osc/request" msg)))

(defvar *bumper-val*)
(defun bumper-echo
  (msg)
  (setq *bumper-val* (send msg :state)))

(defvar *now-pose*)
(defun pose-stamped-echo
  (msg)
  (cond
   ((subclassp (class msg) geometry_msgs::posestamped)
    (setq *now-pose* (send msg :pose)))
   ((subclassp (class msg) geometry_msgs::pose)
    (setq *now-pose* msg))
   ((subclassp (class msg) nav_msgs::odometry)
    (setq *now-pose*
	  (send (send msg :pose) :pose)))))

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
(ros::subscribe "/mobile_base/events/bumper"
		kobuki_msgs::BumperEvent
		#'bumper-echo
		1)

(ros::rate (* 1 *ros2osc-pose-hz*))
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 (cond
  (*bumper-val*
   (print 'echo-bumper)
   (ros2osc_pub
    :ip *osc-host-ip*
    :address "/contact"
    :message (list "controll" *bumper-val*))
   (setq *bumper-val* nil))
  (*now-pose*
   (print 'echo-position)
   (ros2osc_pub
    :ip *osc-host-ip*
    :address "/position"
    :message (list "controll"
		   (send (send *now-pose* :position) :x)
		   (send (send *now-pose* :position) :y)))
   (setq *now-pose* nil))
  )
 ;;(ros::sleep)
 ;; (if *now-pose*
 ;;     (ros2osc_pub
 ;;      :ip *osc-host-ip*
 ;;      :address "/bally"
 ;;      :message (list (send (send *now-pose* :position) :y))))
 )
