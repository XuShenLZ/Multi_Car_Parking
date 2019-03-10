; Auto-generated. Do not edit!


(cl:in-package parking-srv)


;//! \htmlinclude park_maneuver-request.msg.html

(cl:defclass <park_maneuver-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass park_maneuver-request (<park_maneuver-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <park_maneuver-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'park_maneuver-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-srv:<park_maneuver-request> is deprecated: use parking-srv:park_maneuver-request instead.")))

(cl:ensure-generic-function 'Start_lane-val :lambda-list '(m))
(cl:defmethod Start_lane-val ((m <park_maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:Start_lane-val is deprecated.  Use parking-srv:Start_lane instead.")
  (Start_lane m))

(cl:ensure-generic-function 'End_spot-val :lambda-list '(m))
(cl:defmethod End_spot-val ((m <park_maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:End_spot-val is deprecated.  Use parking-srv:End_spot instead.")
  (End_spot m))

(cl:ensure-generic-function 'End_pose-val :lambda-list '(m))
(cl:defmethod End_pose-val ((m <park_maneuver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:End_pose-val is deprecated.  Use parking-srv:End_pose instead.")
  (End_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <park_maneuver-request>) ostream)
  "Serializes a message object of type '<park_maneuver-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <park_maneuver-request>) istream)
  "Deserializes a message object of type '<park_maneuver-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<park_maneuver-request>)))
  "Returns string type for a service object of type '<park_maneuver-request>"
  "parking/park_maneuverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'park_maneuver-request)))
  "Returns string type for a service object of type 'park_maneuver-request"
  "parking/park_maneuverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<park_maneuver-request>)))
  "Returns md5sum for a message object of type '<park_maneuver-request>"
  "a22294d83207313cb1c3be5a262be07d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'park_maneuver-request)))
  "Returns md5sum for a message object of type 'park_maneuver-request"
  "a22294d83207313cb1c3be5a262be07d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<park_maneuver-request>)))
  "Returns full string definition for message of type '<park_maneuver-request>"
  (cl:format cl:nil "string Start_lane~%string End_spot~%string End_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'park_maneuver-request)))
  "Returns full string definition for message of type 'park_maneuver-request"
  (cl:format cl:nil "string Start_lane~%string End_spot~%string End_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <park_maneuver-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Start_lane))
     4 (cl:length (cl:slot-value msg 'End_spot))
     4 (cl:length (cl:slot-value msg 'End_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <park_maneuver-request>))
  "Converts a ROS message object to a list"
  (cl:list 'park_maneuver-request
    (cl:cons ':Start_lane (Start_lane msg))
    (cl:cons ':End_spot (End_spot msg))
    (cl:cons ':End_pose (End_pose msg))
))
;//! \htmlinclude park_maneuver-response.msg.html

(cl:defclass <park_maneuver-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (input
    :reader input
    :initarg :input
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass park_maneuver-response (<park_maneuver-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <park_maneuver-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'park_maneuver-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-srv:<park_maneuver-response> is deprecated: use parking-srv:park_maneuver-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <park_maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:path-val is deprecated.  Use parking-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <park_maneuver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-srv:input-val is deprecated.  Use parking-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <park_maneuver-response>) ostream)
  "Serializes a message object of type '<park_maneuver-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
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
   (cl:slot-value msg 'path))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'input))))
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
   (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <park_maneuver-response>) istream)
  "Deserializes a message object of type '<park_maneuver-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'input) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'input)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<park_maneuver-response>)))
  "Returns string type for a service object of type '<park_maneuver-response>"
  "parking/park_maneuverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'park_maneuver-response)))
  "Returns string type for a service object of type 'park_maneuver-response"
  "parking/park_maneuverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<park_maneuver-response>)))
  "Returns md5sum for a message object of type '<park_maneuver-response>"
  "a22294d83207313cb1c3be5a262be07d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'park_maneuver-response)))
  "Returns md5sum for a message object of type 'park_maneuver-response"
  "a22294d83207313cb1c3be5a262be07d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<park_maneuver-response>)))
  "Returns full string definition for message of type '<park_maneuver-response>"
  (cl:format cl:nil "float64[] path~%float64[] input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'park_maneuver-response)))
  "Returns full string definition for message of type 'park_maneuver-response"
  (cl:format cl:nil "float64[] path~%float64[] input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <park_maneuver-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'input) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <park_maneuver-response>))
  "Converts a ROS message object to a list"
  (cl:list 'park_maneuver-response
    (cl:cons ':path (path msg))
    (cl:cons ':input (input msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'park_maneuver)))
  'park_maneuver-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'park_maneuver)))
  'park_maneuver-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'park_maneuver)))
  "Returns string type for a service object of type '<park_maneuver>"
  "parking/park_maneuver")