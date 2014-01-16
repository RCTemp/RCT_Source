#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "nanopad2twist")

(defun publish-cmd-vel
  (&optional
   (twist *prev-twist*)
   (x (send (send twist :linear) :x))
   (y (send (send twist :linear) :y)))
  (let* ((r (atan2 y x)))
    (send (send twist :linear) :x x)
    (send (send twist :linear) :y y)
    (send (send twist :linear) :z 0)
    (send (send twist :angular) :x 0)
    (send (send twist :angular) :y 0)
    (send (send twist :angular) :z (* -1.0 r))
    (ros::publish "/sphero_node/cmd_vel"
		  twist)))

(defvar *prev-twist* (instance geometry_msgs::twist :init))
(defun joy2twist
  (msg)
  (cond
   ((let (buf
	  (color
	   (list #F(240 0 0) #F(0 240 0) #F(0 0 240)))
	  (cmd-vel
	   (list #'(lambda (tw)
		     (send (send tw :linear) :x
			   (+
			    (- (random 1.0) 0.5)
			    (* 2 (send (send tw :linear) :x))))
		     (send (send tw :linear) :y
			   (+
			    (- (random 1.0) 0.5)
			    (* 2 (send (send tw :linear) :y)))))
		 #'(lambda (tw)
		     (send (send tw :linear) :x
			   (+
			    (- (random 1.0) 0.5)
			    (* -1 (send (send tw :linear) :x))))
		     (send (send tw :linear) :y
			   (+
			    (- (random 1.0) 0.5)
			    (* -1 (send (send tw :linear) :y)))))
		 #'(lambda (tw)
		     (send (send tw :linear) :x
			   (- (random 1.0) 0.5))
		     (send (send tw :linear) :y
			   (- (random 1.0) 0.5)))
		 )))
      (setq
       buf
       (find-if
	#'(lambda (v) (> (car v) 0.1))
	(map cons
	     #'list
	     (send msg :buttons)
	     (apply #'append
		    (make-list (length (send msg :buttons))
			       :initial-element color))
	     (apply #'append
		    (make-list (length (send msg :buttons))
			       :initial-element cmd-vel)))))
      (cond
       (buf
	(if (vectorp (cadr buf))
	    (ros::publish "/sphero_node/cmd_color"
			  (instance std_msgs::ColorRGBA
				    :init
				    :r (aref (cadr buf) 0)
				    :g (aref (cadr buf) 1)
				    :b (aref (cadr buf) 2))))
	(if (functionp (caddr buf))
	    (and (funcall (caddr buf) *prev-twist*)
		 (publish-cmd-vel)))
	t)
       (t nil)))
    (print 'set-color))
   ((or (> (abs (aref (send msg :axes) 0)) 1e-3)
	(> (abs (aref (send msg :axes) 1)) 1e-3))
    (let* ((x (* 2 (- (aref (send msg :axes) 1) 0.5)))
	   (y (* 2 (- (aref (send msg :axes) 0) 0.5))))
      (publish-cmd-vel *prev-twist* x y)))
   ))

(defun collision-detect
  (msg)
  (let* ((twist *prev-twist*)
	 (x (* -1 (send (send twist :linear) :x)))
	 (y (* -1 (send (send twist :linear) :y))))
    (publish-cmd-vel twist x y)))

(ros::advertise "/sphero_node/cmd_vel"
                geometry_msgs::twist)
(ros::advertise "/sphero_node/cmd_color"
                std_msgs::ColorRGBA)

(ros::subscribe "/sphero_node/collision"
		std_msgs::empty
		#'collision-detect
		1)

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
