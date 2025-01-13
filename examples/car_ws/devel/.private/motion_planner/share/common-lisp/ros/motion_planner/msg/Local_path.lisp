; Auto-generated. Do not edit!


(cl:in-package motion_planner-msg)


;//! \htmlinclude Local_path.msg.html

(cl:defclass <Local_path> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ref_state
    :reader ref_state
    :initarg :ref_state
    :type (cl:vector motion_planner-msg:State)
   :initform (cl:make-array 0 :element-type 'motion_planner-msg:State :initial-element (cl:make-instance 'motion_planner-msg:State)))
   (goal_pose
    :reader goal_pose
    :initarg :goal_pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (goal_reached
    :reader goal_reached
    :initarg :goal_reached
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Local_path (<Local_path>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Local_path>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Local_path)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_planner-msg:<Local_path> is deprecated: use motion_planner-msg:Local_path instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Local_path>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planner-msg:header-val is deprecated.  Use motion_planner-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ref_state-val :lambda-list '(m))
(cl:defmethod ref_state-val ((m <Local_path>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planner-msg:ref_state-val is deprecated.  Use motion_planner-msg:ref_state instead.")
  (ref_state m))

(cl:ensure-generic-function 'goal_pose-val :lambda-list '(m))
(cl:defmethod goal_pose-val ((m <Local_path>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planner-msg:goal_pose-val is deprecated.  Use motion_planner-msg:goal_pose instead.")
  (goal_pose m))

(cl:ensure-generic-function 'goal_reached-val :lambda-list '(m))
(cl:defmethod goal_reached-val ((m <Local_path>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planner-msg:goal_reached-val is deprecated.  Use motion_planner-msg:goal_reached instead.")
  (goal_reached m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Local_path>) ostream)
  "Serializes a message object of type '<Local_path>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ref_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ref_state))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'goal_pose))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'goal_reached) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Local_path>) istream)
  "Deserializes a message object of type '<Local_path>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ref_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ref_state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'motion_planner-msg:State))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal_pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal_pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'goal_reached) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Local_path>)))
  "Returns string type for a message object of type '<Local_path>"
  "motion_planner/Local_path")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Local_path)))
  "Returns string type for a message object of type 'Local_path"
  "motion_planner/Local_path")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Local_path>)))
  "Returns md5sum for a message object of type '<Local_path>"
  "c9e6cb19725f08d09811528c99d2a8f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Local_path)))
  "Returns md5sum for a message object of type 'Local_path"
  "c9e6cb19725f08d09811528c99d2a8f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Local_path>)))
  "Returns full string definition for message of type '<Local_path>"
  (cl:format cl:nil "Header header ~%State[] ref_state~%float32[] goal_pose~%bool goal_reached~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motion_planner/State~%float32[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Local_path)))
  "Returns full string definition for message of type 'Local_path"
  (cl:format cl:nil "Header header ~%State[] ref_state~%float32[] goal_pose~%bool goal_reached~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motion_planner/State~%float32[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Local_path>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ref_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Local_path>))
  "Converts a ROS message object to a list"
  (cl:list 'Local_path
    (cl:cons ':header (header msg))
    (cl:cons ':ref_state (ref_state msg))
    (cl:cons ':goal_pose (goal_pose msg))
    (cl:cons ':goal_reached (goal_reached msg))
))
