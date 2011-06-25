; Auto-generated. Do not edit!


(cl:in-package arduino_broadcaster-msg)


;//! \htmlinclude arduino_data.msg.html

(cl:defclass <arduino_data> (roslisp-msg-protocol:ros-message)
  ((ticks
    :reader ticks
    :initarg :ticks
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass arduino_data (<arduino_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arduino_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arduino_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arduino_broadcaster-msg:<arduino_data> is deprecated: use arduino_broadcaster-msg:arduino_data instead.")))

(cl:ensure-generic-function 'ticks-val :lambda-list '(m))
(cl:defmethod ticks-val ((m <arduino_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_broadcaster-msg:ticks-val is deprecated.  Use arduino_broadcaster-msg:ticks instead.")
  (ticks m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <arduino_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arduino_broadcaster-msg:angle-val is deprecated.  Use arduino_broadcaster-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arduino_data>) ostream)
  "Serializes a message object of type '<arduino_data>"
  (cl:let* ((signed (cl:slot-value msg 'ticks)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arduino_data>) istream)
  "Deserializes a message object of type '<arduino_data>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ticks) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arduino_data>)))
  "Returns string type for a message object of type '<arduino_data>"
  "arduino_broadcaster/arduino_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arduino_data)))
  "Returns string type for a message object of type 'arduino_data"
  "arduino_broadcaster/arduino_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arduino_data>)))
  "Returns md5sum for a message object of type '<arduino_data>"
  "11d6147d97d74f9f4899f806fe0f02c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arduino_data)))
  "Returns md5sum for a message object of type 'arduino_data"
  "11d6147d97d74f9f4899f806fe0f02c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arduino_data>)))
  "Returns full string definition for message of type '<arduino_data>"
  (cl:format cl:nil "int16 ticks~%int16 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arduino_data)))
  "Returns full string definition for message of type 'arduino_data"
  (cl:format cl:nil "int16 ticks~%int16 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arduino_data>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arduino_data>))
  "Converts a ROS message object to a list"
  (cl:list 'arduino_data
    (cl:cons ':ticks (ticks msg))
    (cl:cons ':angle (angle msg))
))
