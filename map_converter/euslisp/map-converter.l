#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "map_converter")

(defvar *pose-topic-name*
  (or (unix:getenv "MAPCONV_POSE_TOPIC_NAME") "/odom"))
(defvar *pose-topic-class*
  (eval
   (read-from-string
    (or (unix:getenv "NAPCONV_POSE_TOPIC_CLASS")
	"geometry_msgs::posestamped"))))
(defvar *pose-hz*
  (read-from-string
   (or (unix:getenv "MAPCONV_POSE_HZ") "10")))

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
    (ros::publish
     "/map_converter/converted_xyy"
     (instance std_msgs::float32multiarray :init
	       :data
	       (concatenate float-vector
			    pos (float-vector rot))))))

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
	  2)))))))))

(ros::advertise "/map_converter/converted_xyy"
		std_msgs::float32multiarray 1)
(ros::subscribe "/map_converter/xyy"
		std_msgs::float32multiarray
                #'xyy-echo 1)
(ros::subscribe "/map_converter/posestamped"
		geometry_msgs::posestamped
                #'pose-echo 1)

(ros::rate (* 1 *pose-hz*))
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )

#| roseus map-converter.l  /map_converter/posestamped:=/npc1_node/pose_pub