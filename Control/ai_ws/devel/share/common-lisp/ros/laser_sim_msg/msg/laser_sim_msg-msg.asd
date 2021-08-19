
(cl:in-package :asdf)

(defsystem "laser_sim_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd_vel" :depends-on ("_package_cmd_vel"))
    (:file "_package_cmd_vel" :depends-on ("_package"))
    (:file "point_cloud" :depends-on ("_package_point_cloud"))
    (:file "_package_point_cloud" :depends-on ("_package"))
  ))