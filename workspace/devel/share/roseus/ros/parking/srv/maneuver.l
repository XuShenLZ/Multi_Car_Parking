;; Auto-generated. Do not edit!


(when (boundp 'parking::maneuver)
  (if (not (find-package "PARKING"))
    (make-package "PARKING"))
  (shadow 'maneuver (find-package "PARKING")))
(unless (find-package "PARKING::MANEUVER")
  (make-package "PARKING::MANEUVER"))
(unless (find-package "PARKING::MANEUVERREQUEST")
  (make-package "PARKING::MANEUVERREQUEST"))
(unless (find-package "PARKING::MANEUVERRESPONSE")
  (make-package "PARKING::MANEUVERRESPONSE"))

(in-package "ROS")





(defclass parking::maneuverRequest
  :super ros::object
  :slots (_Start_lane _End_spot _End_pose ))

(defmethod parking::maneuverRequest
  (:init
   (&key
    ((:Start_lane __Start_lane) "")
    ((:End_spot __End_spot) "")
    ((:End_pose __End_pose) "")
    )
   (send-super :init)
   (setq _Start_lane (string __Start_lane))
   (setq _End_spot (string __End_spot))
   (setq _End_pose (string __End_pose))
   self)
  (:Start_lane
   (&optional __Start_lane)
   (if __Start_lane (setq _Start_lane __Start_lane)) _Start_lane)
  (:End_spot
   (&optional __End_spot)
   (if __End_spot (setq _End_spot __End_spot)) _End_spot)
  (:End_pose
   (&optional __End_pose)
   (if __End_pose (setq _End_pose __End_pose)) _End_pose)
  (:serialization-length
   ()
   (+
    ;; string _Start_lane
    4 (length _Start_lane)
    ;; string _End_spot
    4 (length _End_spot)
    ;; string _End_pose
    4 (length _End_pose)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Start_lane
       (write-long (length _Start_lane) s) (princ _Start_lane s)
     ;; string _End_spot
       (write-long (length _End_spot) s) (princ _End_spot s)
     ;; string _End_pose
       (write-long (length _End_pose) s) (princ _End_pose s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Start_lane
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Start_lane (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _End_spot
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _End_spot (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _End_pose
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _End_pose (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass parking::maneuverResponse
  :super ros::object
  :slots (_succeed _path _input _dt ))

(defmethod parking::maneuverResponse
  (:init
   (&key
    ((:succeed __succeed) nil)
    ((:path __path) (instance parking::car_state :init))
    ((:input __input) (instance parking::car_input :init))
    ((:dt __dt) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _succeed __succeed)
   (setq _path __path)
   (setq _input __input)
   (setq _dt __dt)
   self)
  (:succeed
   (&optional __succeed)
   (if __succeed (setq _succeed __succeed)) _succeed)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:serialization-length
   ()
   (+
    ;; bool _succeed
    1
    ;; parking/car_state _path
    (send _path :serialization-length)
    ;; parking/car_input _input
    (send _input :serialization-length)
    ;; float64[] _dt
    (* 8    (length _dt)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _succeed
       (if _succeed (write-byte -1 s) (write-byte 0 s))
     ;; parking/car_state _path
       (send _path :serialize s)
     ;; parking/car_input _input
       (send _input :serialize s)
     ;; float64[] _dt
     (write-long (length _dt) s)
     (dotimes (i (length _dt))
       (sys::poke (elt _dt i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _succeed
     (setq _succeed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; parking/car_state _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; parking/car_input _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;; float64[] _dt
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dt (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dt i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass parking::maneuver
  :super ros::object
  :slots ())

(setf (get parking::maneuver :md5sum-) "946fe324ce1d28d42fa3b8f242a28b5c")
(setf (get parking::maneuver :datatype-) "parking/maneuver")
(setf (get parking::maneuver :request) parking::maneuverRequest)
(setf (get parking::maneuver :response) parking::maneuverResponse)

(defmethod parking::maneuverRequest
  (:response () (instance parking::maneuverResponse :init)))

(setf (get parking::maneuverRequest :md5sum-) "946fe324ce1d28d42fa3b8f242a28b5c")
(setf (get parking::maneuverRequest :datatype-) "parking/maneuverRequest")
(setf (get parking::maneuverRequest :definition-)
      "string Start_lane
string End_spot
string End_pose
---
bool      succeed
car_state path
car_input input
float64[] dt

================================================================================
MSG: parking/car_state
# The states used in vehicle control
# x, y, heading and speed
float64[] x
float64[] y
float64[] psi
float64[] v
================================================================================
MSG: parking/car_input
# The input used in vehicle control
# Steering angle and acceleration
float64[] delta
")

(setf (get parking::maneuverResponse :md5sum-) "946fe324ce1d28d42fa3b8f242a28b5c")
(setf (get parking::maneuverResponse :datatype-) "parking/maneuverResponse")
(setf (get parking::maneuverResponse :definition-)
      "string Start_lane
string End_spot
string End_pose
---
bool      succeed
car_state path
car_input input
float64[] dt

================================================================================
MSG: parking/car_state
# The states used in vehicle control
# x, y, heading and speed
float64[] x
float64[] y
float64[] psi
float64[] v
================================================================================
MSG: parking/car_input
# The input used in vehicle control
# Steering angle and acceleration
float64[] delta
")



(provide :parking/maneuver "946fe324ce1d28d42fa3b8f242a28b5c")


