;; Auto-generated. Do not edit!


(when (boundp 'parking::park_maneuver)
  (if (not (find-package "PARKING"))
    (make-package "PARKING"))
  (shadow 'park_maneuver (find-package "PARKING")))
(unless (find-package "PARKING::PARK_MANEUVER")
  (make-package "PARKING::PARK_MANEUVER"))
(unless (find-package "PARKING::PARK_MANEUVERREQUEST")
  (make-package "PARKING::PARK_MANEUVERREQUEST"))
(unless (find-package "PARKING::PARK_MANEUVERRESPONSE")
  (make-package "PARKING::PARK_MANEUVERRESPONSE"))

(in-package "ROS")





(defclass parking::park_maneuverRequest
  :super ros::object
  :slots (_Start_lane _End_spot _End_pose ))

(defmethod parking::park_maneuverRequest
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

(defclass parking::park_maneuverResponse
  :super ros::object
  :slots (_path _input ))

(defmethod parking::park_maneuverResponse
  (:init
   (&key
    ((:path __path) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:input __input) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _path __path)
   (setq _input __input)
   self)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; float64[] _path
    (* 8    (length _path)) 4
    ;; float64[] _input
    (* 8    (length _input)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _path
     (write-long (length _path) s)
     (dotimes (i (length _path))
       (sys::poke (elt _path i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _input
     (write-long (length _input) s)
     (dotimes (i (length _input))
       (sys::poke (elt _input i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _path i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _input
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _input (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _input i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass parking::park_maneuver
  :super ros::object
  :slots ())

(setf (get parking::park_maneuver :md5sum-) "a22294d83207313cb1c3be5a262be07d")
(setf (get parking::park_maneuver :datatype-) "parking/park_maneuver")
(setf (get parking::park_maneuver :request) parking::park_maneuverRequest)
(setf (get parking::park_maneuver :response) parking::park_maneuverResponse)

(defmethod parking::park_maneuverRequest
  (:response () (instance parking::park_maneuverResponse :init)))

(setf (get parking::park_maneuverRequest :md5sum-) "a22294d83207313cb1c3be5a262be07d")
(setf (get parking::park_maneuverRequest :datatype-) "parking/park_maneuverRequest")
(setf (get parking::park_maneuverRequest :definition-)
      "string Start_lane
string End_spot
string End_pose
---
float64[] path
float64[] input
")

(setf (get parking::park_maneuverResponse :md5sum-) "a22294d83207313cb1c3be5a262be07d")
(setf (get parking::park_maneuverResponse :datatype-) "parking/park_maneuverResponse")
(setf (get parking::park_maneuverResponse :definition-)
      "string Start_lane
string End_spot
string End_pose
---
float64[] path
float64[] input
")



(provide :parking/park_maneuver "a22294d83207313cb1c3be5a262be07d")


