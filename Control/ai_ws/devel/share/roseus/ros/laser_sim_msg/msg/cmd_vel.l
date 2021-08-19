;; Auto-generated. Do not edit!


(when (boundp 'laser_sim_msg::cmd_vel)
  (if (not (find-package "LASER_SIM_MSG"))
    (make-package "LASER_SIM_MSG"))
  (shadow 'cmd_vel (find-package "LASER_SIM_MSG")))
(unless (find-package "LASER_SIM_MSG::CMD_VEL")
  (make-package "LASER_SIM_MSG::CMD_VEL"))

(in-package "ROS")
;;//! \htmlinclude cmd_vel.msg.html


(defclass laser_sim_msg::cmd_vel
  :super ros::object
  :slots (_steering_vel _accel_vel ))

(defmethod laser_sim_msg::cmd_vel
  (:init
   (&key
    ((:steering_vel __steering_vel) 0)
    ((:accel_vel __accel_vel) 0)
    )
   (send-super :init)
   (setq _steering_vel (round __steering_vel))
   (setq _accel_vel (round __accel_vel))
   self)
  (:steering_vel
   (&optional __steering_vel)
   (if __steering_vel (setq _steering_vel __steering_vel)) _steering_vel)
  (:accel_vel
   (&optional __accel_vel)
   (if __accel_vel (setq _accel_vel __accel_vel)) _accel_vel)
  (:serialization-length
   ()
   (+
    ;; int32 _steering_vel
    4
    ;; int32 _accel_vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _steering_vel
       (write-long _steering_vel s)
     ;; int32 _accel_vel
       (write-long _accel_vel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _steering_vel
     (setq _steering_vel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _accel_vel
     (setq _accel_vel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get laser_sim_msg::cmd_vel :md5sum-) "cc66927a56ea8ece1366e10349e9d514")
(setf (get laser_sim_msg::cmd_vel :datatype-) "laser_sim_msg/cmd_vel")
(setf (get laser_sim_msg::cmd_vel :definition-)
      "int32 steering_vel
int32 accel_vel
")



(provide :laser_sim_msg/cmd_vel "cc66927a56ea8ece1366e10349e9d514")


