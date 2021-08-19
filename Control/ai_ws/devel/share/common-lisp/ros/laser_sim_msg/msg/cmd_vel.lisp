; Auto-generated. Do not edit!


(cl:in-package laser_sim_msg-msg)


;//! \htmlinclude cmd_vel.msg.html

(cl:defclass <cmd_vel> (roslisp-msg-protocol:ros-message)
  ((steering_vel
    :reader steering_vel
    :initarg :steering_vel
    :type cl:integer
    :initform 0)
   (accel_vel
    :reader accel_vel
    :initarg :accel_vel
    :type cl:integer
    :initform 0))
)

(cl:defclass cmd_vel (<cmd_vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name laser_sim_msg-msg:<cmd_vel> is deprecated: use laser_sim_msg-msg:cmd_vel instead.")))

(cl:ensure-generic-function 'steering_vel-val :lambda-list '(m))
(cl:defmethod steering_vel-val ((m <cmd_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sim_msg-msg:steering_vel-val is deprecated.  Use laser_sim_msg-msg:steering_vel instead.")
  (steering_vel m))

(cl:ensure-generic-function 'accel_vel-val :lambda-list '(m))
(cl:defmethod accel_vel-val ((m <cmd_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sim_msg-msg:accel_vel-val is deprecated.  Use laser_sim_msg-msg:accel_vel instead.")
  (accel_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_vel>) ostream)
  "Serializes a message object of type '<cmd_vel>"
  (cl:let* ((signed (cl:slot-value msg 'steering_vel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'accel_vel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_vel>) istream)
  "Deserializes a message object of type '<cmd_vel>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering_vel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'accel_vel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_vel>)))
  "Returns string type for a message object of type '<cmd_vel>"
  "laser_sim_msg/cmd_vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_vel)))
  "Returns string type for a message object of type 'cmd_vel"
  "laser_sim_msg/cmd_vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_vel>)))
  "Returns md5sum for a message object of type '<cmd_vel>"
  "cc66927a56ea8ece1366e10349e9d514")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_vel)))
  "Returns md5sum for a message object of type 'cmd_vel"
  "cc66927a56ea8ece1366e10349e9d514")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_vel>)))
  "Returns full string definition for message of type '<cmd_vel>"
  (cl:format cl:nil "int32 steering_vel~%int32 accel_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_vel)))
  "Returns full string definition for message of type 'cmd_vel"
  (cl:format cl:nil "int32 steering_vel~%int32 accel_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_vel>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_vel>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_vel
    (cl:cons ':steering_vel (steering_vel msg))
    (cl:cons ':accel_vel (accel_vel msg))
))
