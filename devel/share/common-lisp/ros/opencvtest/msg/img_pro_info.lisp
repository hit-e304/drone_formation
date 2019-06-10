; Auto-generated. Do not edit!


(cl:in-package opencvtest-msg)


;//! \htmlinclude img_pro_info.msg.html

(cl:defclass <img_pro_info> (roslisp-msg-protocol:ros-message)
  ((find_obs_flag
    :reader find_obs_flag
    :initarg :find_obs_flag
    :type cl:boolean
    :initform cl:nil)
   (dis
    :reader dis
    :initarg :dis
    :type cl:float
    :initform 0.0)
   (pos_left
    :reader pos_left
    :initarg :pos_left
    :type cl:integer
    :initform 0)
   (pos_right
    :reader pos_right
    :initarg :pos_right
    :type cl:integer
    :initform 0)
   (x_pos
    :reader x_pos
    :initarg :x_pos
    :type cl:integer
    :initform 0)
   (y_pos
    :reader y_pos
    :initarg :y_pos
    :type cl:integer
    :initform 0))
)

(cl:defclass img_pro_info (<img_pro_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <img_pro_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'img_pro_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencvtest-msg:<img_pro_info> is deprecated: use opencvtest-msg:img_pro_info instead.")))

(cl:ensure-generic-function 'find_obs_flag-val :lambda-list '(m))
(cl:defmethod find_obs_flag-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:find_obs_flag-val is deprecated.  Use opencvtest-msg:find_obs_flag instead.")
  (find_obs_flag m))

(cl:ensure-generic-function 'dis-val :lambda-list '(m))
(cl:defmethod dis-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:dis-val is deprecated.  Use opencvtest-msg:dis instead.")
  (dis m))

(cl:ensure-generic-function 'pos_left-val :lambda-list '(m))
(cl:defmethod pos_left-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:pos_left-val is deprecated.  Use opencvtest-msg:pos_left instead.")
  (pos_left m))

(cl:ensure-generic-function 'pos_right-val :lambda-list '(m))
(cl:defmethod pos_right-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:pos_right-val is deprecated.  Use opencvtest-msg:pos_right instead.")
  (pos_right m))

(cl:ensure-generic-function 'x_pos-val :lambda-list '(m))
(cl:defmethod x_pos-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:x_pos-val is deprecated.  Use opencvtest-msg:x_pos instead.")
  (x_pos m))

(cl:ensure-generic-function 'y_pos-val :lambda-list '(m))
(cl:defmethod y_pos-val ((m <img_pro_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:y_pos-val is deprecated.  Use opencvtest-msg:y_pos instead.")
  (y_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <img_pro_info>) ostream)
  "Serializes a message object of type '<img_pro_info>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'find_obs_flag) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'pos_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pos_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <img_pro_info>) istream)
  "Deserializes a message object of type '<img_pro_info>"
    (cl:setf (cl:slot-value msg 'find_obs_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dis) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_pos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_pos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<img_pro_info>)))
  "Returns string type for a message object of type '<img_pro_info>"
  "opencvtest/img_pro_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'img_pro_info)))
  "Returns string type for a message object of type 'img_pro_info"
  "opencvtest/img_pro_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<img_pro_info>)))
  "Returns md5sum for a message object of type '<img_pro_info>"
  "254e05080fb643551976301900a3b655")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'img_pro_info)))
  "Returns md5sum for a message object of type 'img_pro_info"
  "254e05080fb643551976301900a3b655")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<img_pro_info>)))
  "Returns full string definition for message of type '<img_pro_info>"
  (cl:format cl:nil "    bool find_obs_flag~%    float64 dis~%    int32 pos_left~%    int32 pos_right~%    int32 x_pos~%    int32 y_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'img_pro_info)))
  "Returns full string definition for message of type 'img_pro_info"
  (cl:format cl:nil "    bool find_obs_flag~%    float64 dis~%    int32 pos_left~%    int32 pos_right~%    int32 x_pos~%    int32 y_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <img_pro_info>))
  (cl:+ 0
     1
     8
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <img_pro_info>))
  "Converts a ROS message object to a list"
  (cl:list 'img_pro_info
    (cl:cons ':find_obs_flag (find_obs_flag msg))
    (cl:cons ':dis (dis msg))
    (cl:cons ':pos_left (pos_left msg))
    (cl:cons ':pos_right (pos_right msg))
    (cl:cons ':x_pos (x_pos msg))
    (cl:cons ':y_pos (y_pos msg))
))
