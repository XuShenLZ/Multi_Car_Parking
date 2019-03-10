; Auto-generated. Do not edit!


(cl:in-package parking-srv)


;//! \htmlinclude maneuver-request.msg.html

(cl:defclass <maneuver-request> (roslisp-msg-protocol:ros-message)
  ((Start_lane
    :reader Start_lane
    :initarg :Start_lane
    :type cl:string
    :initform "")
   (End_spot
    :reader End_spot
    :initarg :End_spot
    :type cl:string
    :initform "")
   (End_pose
    :reader End_pose
    :initarg :End_pose
    :type cl:string
    :initform ""))
)

(cl:defclass maneuver-request (<maneuver-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <maneuver-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'maneuver-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-srv:<maneuver-request> is deprecated: use parking-srv:maneuver-request instead.")))

(cl:ensure-generic-function 'Start_lane-val :lambda-list '(m))
(cl:defmethod Start_lane-val ((m <maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:Start_lane-val is deprecated.  Use parking-srv:Start_lane instead.")
  (Start_lane m))

(cl:ensure-generic-function 'End_spot-val :lambda-list '(m))
(cl:defmethod End_spot-val ((m <maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:End_spot-val is deprecated.  Use parking-srv:End_spot instead.")
  (End_spot m))

(cl:ensure-generic-function 'End_pose-val :lambda-list '(m))
(cl:defmethod End_pose-val ((m <maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:End_pose-val is deprecated.  Use parking-srv:End_pose instead.")
  (End_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <maneuver-request>) ostream)
  "Serializes a message object of type '<maneuver-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Start_lane))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Start_lane))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'End_spot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'End_spot))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'End_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'End_pose))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <maneuver-request>) istream)
  "Deserializes a message object of type '<maneuver-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Start_lane) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Start_lane) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'End_spot) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'End_spot) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'End_pose) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'End_pose) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<maneuver-request>)))
  "Returns string type for a service object of type '<maneuver-request>"
  "parking/maneuverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'maneuver-request)))
  "Returns string type for a service object of type 'maneuver-request"
  "parking/maneuverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<maneuver-request>)))
  "Returns md5sum for a message object of type '<maneuver-request>"
  "946fe324ce1d28d42fa3b8f242a28b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'maneuver-request)))
  "Returns md5sum for a message object of type 'maneuver-request"
  "946fe324ce1d28d42fa3b8f242a28b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<maneuver-request>)))
  "Returns full string definition for message of type '<maneuver-request>"
  (cl:format cl:nil "string Start_lane~%string End_spot~%string End_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'maneuver-request)))
  "Returns full string definition for message of type 'maneuver-request"
  (cl:format cl:nil "string Start_lane~%string End_spot~%string End_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <maneuver-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Start_lane))
     4 (cl:length (cl:slot-value msg 'End_spot))
     4 (cl:length (cl:slot-value msg 'End_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <maneuver-request>))
  "Converts a ROS message object to a list"
  (cl:list 'maneuver-request
    (cl:cons ':Start_lane (Start_lane msg))
    (cl:cons ':End_spot (End_spot msg))
    (cl:cons ':End_pose (End_pose msg))
))
;//! \htmlinclude maneuver-response.msg.html

(cl:defclass <maneuver-response> (roslisp-msg-protocol:ros-message)
  ((succeed
    :reader succeed
    :initarg :succeed
    :type cl:boolean
    :initform cl:nil)
   (path
    :reader path
    :initarg :path
    :type parking-msg:car_state
    :initform (cl:make-instance 'parking-msg:car_state))
   (input
    :reader input
    :initarg :input
    :type parking-msg:car_input
    :initform (cl:make-instance 'parking-msg:car_input))
   (dt
    :reader dt
    :initarg :dt
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass maneuver-response (<maneuver-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <maneuver-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'maneuver-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-srv:<maneuver-response> is deprecated: use parking-srv:maneuver-response instead.")))

(cl:ensure-generic-function 'succeed-val :lambda-list '(m))
(cl:defmethod succeed-val ((m <maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:succeed-val is deprecated.  Use parking-srv:succeed instead.")
  (succeed m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:path-val is deprecated.  Use parking-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:input-val is deprecated.  Use parking-srv:input instead.")
  (input m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:dt-val is deprecated.  Use parking-srv:dt instead.")
  (dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <maneuver-response>) ostream)
  "Serializes a message object of type '<maneuver-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeed) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'dt))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <maneuver-response>) istream)
  "Deserializes a message object of type '<maneuver-response>"
    (cl:setf (cl:slot-value msg 'succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dt) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dt)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<maneuver-response>)))
  "Returns string type for a service object of type '<maneuver-response>"
  "parking/maneuverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'maneuver-response)))
  "Returns string type for a service object of type 'maneuver-response"
  "parking/maneuverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<maneuver-response>)))
  "Returns md5sum for a message object of type '<maneuver-response>"
  "946fe324ce1d28d42fa3b8f242a28b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'maneuver-response)))
  "Returns md5sum for a message object of type 'maneuver-response"
  "946fe324ce1d28d42fa3b8f242a28b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<maneuver-response>)))
  "Returns full string definition for message of type '<maneuver-response>"
  (cl:format cl:nil "bool      succeed~%car_state path~%car_input input~%float64[] dt~%~%================================================================================~%MSG: parking/car_state~%# The states used in vehicle control~%# x, y, heading and speed~%float64[] x~%float64[] y~%float64[] psi~%float64[] v~%================================================================================~%MSG: parking/car_input~%# The input used in vehicle control~%# Steering angle and acceleration~%float64[] delta~%float64[] acc~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'maneuver-response)))
  "Returns full string definition for message of type 'maneuver-response"
  (cl:format cl:nil "bool      succeed~%car_state path~%car_input input~%float64[] dt~%~%================================================================================~%MSG: parking/car_state~%# The states used in vehicle control~%# x, y, heading and speed~%float64[] x~%float64[] y~%float64[] psi~%float64[] v~%================================================================================~%MSG: parking/car_input~%# The input used in vehicle control~%# Steering angle and acceleration~%float64[] delta~%float64[] acc~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <maneuver-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <maneuver-response>))
  "Converts a ROS message object to a list"
  (cl:list 'maneuver-response
    (cl:cons ':succeed (succeed msg))
    (cl:cons ':path (path msg))
    (cl:cons ':input (input msg))
    (cl:cons ':dt (dt msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'maneuver)))
  'maneuver-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'maneuver)))
  'maneuver-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'maneuver)))
  "Returns string type for a service object of type '<maneuver>"
  "parking/maneuver")