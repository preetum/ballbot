; Auto-generated. Do not edit!


(cl:in-package ros_ard_to_fro-msg)


;//! \htmlinclude odom_data.msg.html

(cl:defclass <odom_data> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass odom_data (<odom_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <odom_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'odom_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_ard_to_fro-msg:<odom_data> is deprecated: use ros_ard_to_fro-msg:odom_data instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <odom_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ard_to_fro-msg:x-val is deprecated.  Use ros_ard_to_fro-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <odom_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ard_to_fro-msg:y-val is deprecated.  Use ros_ard_to_fro-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <odom_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ard_to_fro-msg:dist-val is deprecated.  Use ros_ard_to_fro-msg:dist instead.")
  (dist m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <odom_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ard_to_fro-msg:angle-val is deprecated.  Use ros_ard_to_fro-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <odom_data>) ostream)
  "Serializes a message object of type '<odom_data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <odom_data>) istream)
  "Deserializes a message object of type '<odom_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<odom_data>)))
  "Returns string type for a message object of type '<odom_data>"
  "ros_ard_to_fro/odom_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'odom_data)))
  "Returns string type for a message object of type 'odom_data"
  "ros_ard_to_fro/odom_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<odom_data>)))
  "Returns md5sum for a message object of type '<odom_data>"
  "eb075f195d7b4ffca1d8fa2c45908892")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'odom_data)))
  "Returns md5sum for a message object of type 'odom_data"
  "eb075f195d7b4ffca1d8fa2c45908892")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<odom_data>)))
  "Returns full string definition for message of type '<odom_data>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 dist~%int16 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'odom_data)))
  "Returns full string definition for message of type 'odom_data"
  (cl:format cl:nil "float32 x~%float32 y~%float32 dist~%int16 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <odom_data>))
  (cl:+ 0
     4
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <odom_data>))
  "Converts a ROS message object to a list"
  (cl:list 'odom_data
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':dist (dist msg))
    (cl:cons ':angle (angle msg))
))
