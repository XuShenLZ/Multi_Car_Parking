;; Auto-generated. Do not edit!


(when (boundp 'parking::cost_map)
  (if (not (find-package "PARKING"))
    (make-package "PARKING"))
  (shadow 'cost_map (find-package "PARKING")))
(unless (find-package "PARKING::COST_MAP")
  (make-package "PARKING::COST_MAP"))

(in-package "ROS")
;;//! \htmlinclude cost_map.msg.html


(defclass parking::cost_map
  :super ros::object
  :slots (_length _width _time _data ))

(defmethod parking::cost_map
  (:init
   (&key
    ((:length __length) 0)
    ((:width __width) 0)
    ((:time __time) 0)
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _length (round __length))
   (setq _width (round __width))
   (setq _time (round __time))
   (setq _data __data)
   self)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16 _length
    2
    ;; int16 _width
    2
    ;; int16 _time
    2
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _length
       (write-word _length s)
     ;; int16 _width
       (write-word _width s)
     ;; int16 _time
       (write-word _time s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _length
     (setq _length (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _time
     (setq _time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get parking::cost_map :md5sum-) "be54434a940ad22ca7ed96d2870a90a3")
(setf (get parking::cost_map :datatype-) "parking/cost_map")
(setf (get parking::cost_map :definition-)
      "# The msg type for sending cost map information
# length is the dimension on x-direction
# width is the dimension on y-direction
# time is the time stamp
# data is the map dictionary transformed to 1-D vector

int16     length
int16     width
int16     time
float64[] data
")



(provide :parking/cost_map "be54434a940ad22ca7ed96d2870a90a3")


