; Auto-generated. Do not edit!


(cl:in-package laser_sim_msg-msg)


;//! \htmlinclude point_cloud.msg.html

(cl:defclass <point_cloud> (roslisp-msg-protocol:ros-message)
  ((ranges
    :reader ranges
    :initarg :ranges
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (channel
    :reader channel
    :initarg :channel
    :type cl:integer
    :initform 0))
)

(cl:defclass point_cloud (<point_cloud>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point_cloud>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point_cloud)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name laser_sim_msg-msg:<point_cloud> is deprecated: use laser_sim_msg-msg:point_cloud instead.")))

(cl:ensure-generic-function 'ranges-val :lambda-list '(m))
(cl:defmethod ranges-val ((m <point_cloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sim_msg-msg:ranges-val is deprecated.  Use laser_sim_msg-msg:ranges instead.")
  (ranges m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <point_cloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sim_msg-msg:size-val is deprecated.  Use laser_sim_msg-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <point_cloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sim_msg-msg:channel-val is deprecated.  Use laser_sim_msg-msg:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point_cloud>) ostream)
  "Serializes a message object of type '<point_cloud>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ranges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'ranges))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channel)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point_cloud>) istream)
  "Deserializes a message object of type '<point_cloud>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ranges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ranges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channel)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point_cloud>)))
  "Returns string type for a message object of type '<point_cloud>"
  "laser_sim_msg/point_cloud")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point_cloud)))
  "Returns string type for a message object of type 'point_cloud"
  "laser_sim_msg/point_cloud")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point_cloud>)))
  "Returns md5sum for a message object of type '<point_cloud>"
  "8c9f5b31aed2b325faeea7a6697b7f64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point_cloud)))
  "Returns md5sum for a message object of type 'point_cloud"
  "8c9f5b31aed2b325faeea7a6697b7f64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point_cloud>)))
  "Returns full string definition for message of type '<point_cloud>"
  (cl:format cl:nil "float32[] ranges~%uint32 size~%uint32 channel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point_cloud)))
  "Returns full string definition for message of type 'point_cloud"
  (cl:format cl:nil "float32[] ranges~%uint32 size~%uint32 channel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point_cloud>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ranges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point_cloud>))
  "Converts a ROS message object to a list"
  (cl:list 'point_cloud
    (cl:cons ':ranges (ranges msg))
    (cl:cons ':size (size msg))
    (cl:cons ':channel (channel msg))
))
