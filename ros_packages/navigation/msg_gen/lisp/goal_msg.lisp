; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude goal_msg.msg.html

(cl:defclass <goal_msg> (roslisp-msg-protocol:ros-message)
  ((d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
   (th
    :reader th
    :initarg :th
    :type cl:float
    :initform 0.0)
   (opt
    :reader opt
    :initarg :opt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass goal_msg (<goal_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <goal_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'goal_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<goal_msg> is deprecated: use navigation-msg:goal_msg instead.")))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <goal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:d-val is deprecated.  Use navigation-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'th-val :lambda-list '(m))
(cl:defmethod th-val ((m <goal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:th-val is deprecated.  Use navigation-msg:th instead.")
  (th m))

(cl:ensure-generic-function 'opt-val :lambda-list '(m))
(cl:defmethod opt-val ((m <goal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:opt-val is deprecated.  Use navigation-msg:opt instead.")
  (opt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <goal_msg>) ostream)
  "Serializes a message object of type '<goal_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'opt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <goal_msg>) istream)
  "Deserializes a message object of type '<goal_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'th) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opt) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<goal_msg>)))
  "Returns string type for a message object of type '<goal_msg>"
  "navigation/goal_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'goal_msg)))
  "Returns string type for a message object of type 'goal_msg"
  "navigation/goal_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<goal_msg>)))
  "Returns md5sum for a message object of type '<goal_msg>"
  "ead0a80715facf19ee84694b8322c9ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'goal_msg)))
  "Returns md5sum for a message object of type 'goal_msg"
  "ead0a80715facf19ee84694b8322c9ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<goal_msg>)))
  "Returns full string definition for message of type '<goal_msg>"
  (cl:format cl:nil "float32 d~%float32 th~%int16 opt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'goal_msg)))
  "Returns full string definition for message of type 'goal_msg"
  (cl:format cl:nil "float32 d~%float32 th~%int16 opt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <goal_msg>))
  (cl:+ 0
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <goal_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'goal_msg
    (cl:cons ':d (d msg))
    (cl:cons ':th (th msg))
    (cl:cons ':opt (opt msg))
))
