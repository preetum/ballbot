; Auto-generated. Do not edit!


(cl:in-package ros_to_arduino_control-msg)


;//! \htmlinclude drive_cmd.msg.html

(cl:defclass <drive_cmd> (roslisp-msg-protocol:ros-message)
  ((drive_speed
    :reader drive_speed
    :initarg :drive_speed
    :type cl:fixnum
    :initform 0)
   (steer_angle
    :reader steer_angle
    :initarg :steer_angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass drive_cmd (<drive_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drive_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drive_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_to_arduino_control-msg:<drive_cmd> is deprecated: use ros_to_arduino_control-msg:drive_cmd instead.")))

(cl:ensure-generic-function 'drive_speed-val :lambda-list '(m))
(cl:defmethod drive_speed-val ((m <drive_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_to_arduino_control-msg:drive_speed-val is deprecated.  Use ros_to_arduino_control-msg:drive_speed instead.")
  (drive_speed m))

(cl:ensure-generic-function 'steer_angle-val :lambda-list '(m))
(cl:defmethod steer_angle-val ((m <drive_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_to_arduino_control-msg:steer_angle-val is deprecated.  Use ros_to_arduino_control-msg:steer_angle instead.")
  (steer_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drive_cmd>) ostream)
  "Serializes a message object of type '<drive_cmd>"
  (cl:let* ((signed (cl:slot-value msg 'drive_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steer_angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drive_cmd>) istream)
  "Deserializes a message object of type '<drive_cmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drive_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steer_angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drive_cmd>)))
  "Returns string type for a message object of type '<drive_cmd>"
  "ros_to_arduino_control/drive_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drive_cmd)))
  "Returns string type for a message object of type 'drive_cmd"
  "ros_to_arduino_control/drive_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drive_cmd>)))
  "Returns md5sum for a message object of type '<drive_cmd>"
  "20d51325e4a2e2c39f4c2497c2070be5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drive_cmd)))
  "Returns md5sum for a message object of type 'drive_cmd"
  "20d51325e4a2e2c39f4c2497c2070be5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drive_cmd>)))
  "Returns full string definition for message of type '<drive_cmd>"
  (cl:format cl:nil "int16 drive_speed~%int16 steer_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drive_cmd)))
  "Returns full string definition for message of type 'drive_cmd"
  (cl:format cl:nil "int16 drive_speed~%int16 steer_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drive_cmd>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drive_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'drive_cmd
    (cl:cons ':drive_speed (drive_speed msg))
    (cl:cons ':steer_angle (steer_angle msg))
))
