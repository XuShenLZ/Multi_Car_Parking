; Auto-generated. Do not edit!


(cl:in-package parking-msg)


;//! \htmlinclude car_input.msg.html

(cl:defclass <car_input> (roslisp-msg-protocol:ros-message)
  ((delta
    :reader delta
    :initarg :delta
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass car_input (<car_input>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <car_input>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'car_input)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-msg:<car_input> is deprecated: use parking-msg:car_input instead.")))

(cl:ensure-generic-function 'delta-val :lambda-list '(m))
(cl:defmethod delta-val ((m <car_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:delta-val is deprecated.  Use parking-msg:delta instead.")
  (delta m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <car_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:acc-val is deprecated.  Use parking-msg:acc instead.")
  (acc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <car_input>) ostream)
  "Serializes a message object of type '<car_input>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'delta))))
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
   (cl:slot-value msg 'delta))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'acc))))
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
   (cl:slot-value msg 'acc))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <car_input>) istream)
  "Deserializes a message object of type '<car_input>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'delta) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'delta)))
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
  (cl:setf (cl:slot-value msg 'acc) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'acc)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<car_input>)))
  "Returns string type for a message object of type '<car_input>"
  "parking/car_input")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'car_input)))
  "Returns string type for a message object of type 'car_input"
  "parking/car_input")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<car_input>)))
  "Returns md5sum for a message object of type '<car_input>"
  "abdc68e560ea7f4923b164eecfe49022")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'car_input)))
  "Returns md5sum for a message object of type 'car_input"
  "abdc68e560ea7f4923b164eecfe49022")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<car_input>)))
  "Returns full string definition for message of type '<car_input>"
  (cl:format cl:nil "# The input used in vehicle control~%# Steering angle and acceleration~%float64[] delta~%float64[] acc~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'car_input)))
  "Returns full string definition for message of type 'car_input"
  (cl:format cl:nil "# The input used in vehicle control~%# Steering angle and acceleration~%float64[] delta~%float64[] acc~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <car_input>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'delta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'acc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <car_input>))
  "Converts a ROS message object to a list"
  (cl:list 'car_input
    (cl:cons ':delta (delta msg))
    (cl:cons ':acc (acc msg))
))
