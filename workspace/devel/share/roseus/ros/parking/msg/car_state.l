;; Auto-generated. Do not edit!


(when (boundp 'parking::car_state)
  (if (not (find-package "PARKING"))
    (make-package "PARKING"))
  (shadow 'car_state (find-package "PARKING")))
(unless (find-package "PARKING::CAR_STATE")
  (make-package "PARKING::CAR_STATE"))

(in-package "ROS")
;;//! \htmlinclude car_state.msg.html


(defclass parking::car_state
  :super ros::object
  :slots (_x _y _psi _v ))

(defmethod parking::car_state
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:psi __psi) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:v __v) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _psi __psi)
   (setq _v __v)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; float64[] _x
    (* 8    (length _x)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ;; float64[] _psi
    (* 8    (length _psi)) 4
    ;; float64[] _v
    (* 8    (length _v)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _psi
     (write-long (length _psi) s)
     (dotimes (i (length _psi))
       (sys::poke (elt _psi i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _v
     (write-long (length _v) s)
     (dotimes (i (length _v))
       (sys::poke (elt _v i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _psi
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _psi (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _psi i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _v
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _v (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _v i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get parking::car_state :md5sum-) "46770b25fddd17772c0c599a8d50502a")
(setf (get parking::car_state :datatype-) "parking/car_state")
(setf (get parking::car_state :definition-)
      "# The states used in vehicle control
# x, y, heading and speed
float64[] x
float64[] y
float64[] psi
float64[] v
")



(provide :parking/car_state "46770b25fddd17772c0c599a8d50502a")


