;; Auto-generated. Do not edit!


(when (boundp 'motion_planner::Local_path)
  (if (not (find-package "MOTION_PLANNER"))
    (make-package "MOTION_PLANNER"))
  (shadow 'Local_path (find-package "MOTION_PLANNER")))
(unless (find-package "MOTION_PLANNER::LOCAL_PATH")
  (make-package "MOTION_PLANNER::LOCAL_PATH"))

(in-package "ROS")
;;//! \htmlinclude Local_path.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass motion_planner::Local_path
  :super ros::object
  :slots (_header _ref_state _goal_pose _goal_reached ))

(defmethod motion_planner::Local_path
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ref_state __ref_state) (let (r) (dotimes (i 0) (push (instance motion_planner::State :init) r)) r))
    ((:goal_pose __goal_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:goal_reached __goal_reached) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ref_state __ref_state)
   (setq _goal_pose __goal_pose)
   (setq _goal_reached __goal_reached)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ref_state
   (&rest __ref_state)
   (if (keywordp (car __ref_state))
       (send* _ref_state __ref_state)
     (progn
       (if __ref_state (setq _ref_state (car __ref_state)))
       _ref_state)))
  (:goal_pose
   (&optional __goal_pose)
   (if __goal_pose (setq _goal_pose __goal_pose)) _goal_pose)
  (:goal_reached
   (&optional __goal_reached)
   (if __goal_reached (setq _goal_reached __goal_reached)) _goal_reached)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; motion_planner/State[] _ref_state
    (apply #'+ (send-all _ref_state :serialization-length)) 4
    ;; float32[] _goal_pose
    (* 4    (length _goal_pose)) 4
    ;; bool _goal_reached
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; motion_planner/State[] _ref_state
     (write-long (length _ref_state) s)
     (dolist (elem _ref_state)
       (send elem :serialize s)
       )
     ;; float32[] _goal_pose
     (write-long (length _goal_pose) s)
     (dotimes (i (length _goal_pose))
       (sys::poke (elt _goal_pose i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _goal_reached
       (if _goal_reached (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; motion_planner/State[] _ref_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ref_state (let (r) (dotimes (i n) (push (instance motion_planner::State :init) r)) r))
     (dolist (elem- _ref_state)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _goal_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goal_pose i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _goal_reached
     (setq _goal_reached (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get motion_planner::Local_path :md5sum-) "c9e6cb19725f08d09811528c99d2a8f5")
(setf (get motion_planner::Local_path :datatype-) "motion_planner/Local_path")
(setf (get motion_planner::Local_path :definition-)
      "Header header 
State[] ref_state
float32[] goal_pose
bool goal_reached
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: motion_planner/State
float32[] data
")



(provide :motion_planner/Local_path "c9e6cb19725f08d09811528c99d2a8f5")


