;; Auto-generated. Do not edit!


(when (boundp 'laser_sim_msg::point_cloud)
  (if (not (find-package "LASER_SIM_MSG"))
    (make-package "LASER_SIM_MSG"))
  (shadow 'point_cloud (find-package "LASER_SIM_MSG")))
(unless (find-package "LASER_SIM_MSG::POINT_CLOUD")
  (make-package "LASER_SIM_MSG::POINT_CLOUD"))

(in-package "ROS")
;;//! \htmlinclude point_cloud.msg.html


(defclass laser_sim_msg::point_cloud
  :super ros::object
  :slots (_ranges _size _channel ))

(defmethod laser_sim_msg::point_cloud
  (:init
   (&key
    ((:ranges __ranges) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:size __size) 0)
    ((:channel __channel) 0)
    )
   (send-super :init)
   (setq _ranges __ranges)
   (setq _size (round __size))
   (setq _channel (round __channel))
   self)
  (:ranges
   (&optional __ranges)
   (if __ranges (setq _ranges __ranges)) _ranges)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; float32[] _ranges
    (* 4    (length _ranges)) 4
    ;; uint32 _size
    4
    ;; uint32 _channel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _ranges
     (write-long (length _ranges) s)
     (dotimes (i (length _ranges))
       (sys::poke (elt _ranges i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _size
       (write-long _size s)
     ;; uint32 _channel
       (write-long _channel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _ranges
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ranges (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ranges i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _channel
     (setq _channel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get laser_sim_msg::point_cloud :md5sum-) "8c9f5b31aed2b325faeea7a6697b7f64")
(setf (get laser_sim_msg::point_cloud :datatype-) "laser_sim_msg/point_cloud")
(setf (get laser_sim_msg::point_cloud :definition-)
      "float32[] ranges
uint32 size
uint32 channel
")



(provide :laser_sim_msg/point_cloud "8c9f5b31aed2b325faeea7a6697b7f64")


