; Auto-generated. Do not edit!


(cl:in-package parking-msg)


;//! \htmlinclude cost_map.msg.html

(cl:defclass <cost_map> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass cost_map (<cost_map>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cost_map>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cost_map)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parking-msg:<cost_map> is deprecated: use parking-msg:cost_map instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <cost_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:length-val is deprecated.  Use parking-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <cost_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:width-val is deprecated.  Use parking-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <cost_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:time-val is deprecated.  Use parking-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <cost_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parking-msg:data-val is deprecated.  Use parking-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cost_map>) ostream)
  "Serializes a message object of type '<cost_map>"
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
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
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cost_map>) istream)
  "Deserializes a message object of type '<cost_map>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cost_map>)))
  "Returns string type for a message object of type '<cost_map>"
  "parking/cost_map")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cost_map)))
  "Returns string type for a message object of type 'cost_map"
  "parking/cost_map")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cost_map>)))
  "Returns md5sum for a message object of type '<cost_map>"
  "be54434a940ad22ca7ed96d2870a90a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cost_map)))
  "Returns md5sum for a message object of type 'cost_map"
  "be54434a940ad22ca7ed96d2870a90a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cost_map>)))
  "Returns full string definition for message of type '<cost_map>"
  (cl:format cl:nil "# The msg type for sending cost map information~%# length is the dimension on x-direction~%# width is the dimension on y-direction~%# time is the time stamp~%# data is the map dictionary transformed to 1-D vector~%~%int16     length~%int16     width~%int16     time~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cost_map)))
  "Returns full string definition for message of type 'cost_map"
  (cl:format cl:nil "# The msg type for sending cost map information~%# length is the dimension on x-direction~%# width is the dimension on y-direction~%# time is the time stamp~%# data is the map dictionary transformed to 1-D vector~%~%int16     length~%int16     width~%int16     time~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cost_map>))
  (cl:+ 0
     2
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cost_map>))
  "Converts a ROS message object to a list"
  (cl:list 'cost_map
    (cl:cons ':length (length msg))
    (cl:cons ':width (width msg))
    (cl:cons ':time (time msg))
    (cl:cons ':data (data msg))
))
