;; Auto-generated. Do not edit!


(when (boundp 'parking::car_input)
  (if (not (find-package "PARKING"))
    (make-package "PARKING"))
  (shadow 'car_input (find-package "PARKING")))
(unless (find-package "PARKING::CAR_INPUT")
  (make-package "PARKING::CAR_INPUT"))

(in-package "ROS")
;;//! \htmlinclude car_input.msg.html


(defclass parking::car_input
  :super ros::object
  :slots (_delta _acc ))

(defmethod parking::car_input
  (:init
   (&key
    ((:delta __delta) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _delta __delta)
   (setq _acc __acc)
   self)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:serialization-length
   ()
   (+
    ;; float64[] _delta
    (* 8    (length _delta)) 4
    ;; float64[] _acc
    (* 8    (length _acc)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _delta
     (write-long (length _delta) s)
     (dotimes (i (length _delta))
       (sys::poke (elt _delta i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _acc
     (write-long (length _acc) s)
     (dotimes (i (length _acc))
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _delta
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _delta (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _delta i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _acc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get parking::car_input :md5sum-) "abdc68e560ea7f4923b164eecfe49022")
(setf (get parking::car_input :datatype-) "parking/car_input")
(setf (get parking::car_input :definition-)
      "# The input used in vehicle control
# Steering angle and acceleration
float64[] delta
float64[] acc
")



(provide :parking/car_input "abdc68e560ea7f4923b164eecfe49022")


