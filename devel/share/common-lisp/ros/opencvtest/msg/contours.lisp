; Auto-generated. Do not edit!


(cl:in-package opencvtest-msg)


;//! \htmlinclude contours.msg.html

(cl:defclass <contours> (roslisp-msg-protocol:ros-message)
  ((pro_info
    :reader pro_info
    :initarg :pro_info
    :type opencvtest-msg:img_pro_info
    :initform (cl:make-instance 'opencvtest-msg:img_pro_info)))
)

(cl:defclass contours (<contours>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <contours>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'contours)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencvtest-msg:<contours> is deprecated: use opencvtest-msg:contours instead.")))

(cl:ensure-generic-function 'pro_info-val :lambda-list '(m))
(cl:defmethod pro_info-val ((m <contours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencvtest-msg:pro_info-val is deprecated.  Use opencvtest-msg:pro_info instead.")
  (pro_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <contours>) ostream)
  "Serializes a message object of type '<contours>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pro_info) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <contours>) istream)
  "Deserializes a message object of type '<contours>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pro_info) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<contours>)))
  "Returns string type for a message object of type '<contours>"
  "opencvtest/contours")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'contours)))
  "Returns string type for a message object of type 'contours"
  "opencvtest/contours")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<contours>)))
  "Returns md5sum for a message object of type '<contours>"
  "80338e53bc763a94f85a1a487d6d2c70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'contours)))
  "Returns md5sum for a message object of type 'contours"
  "80338e53bc763a94f85a1a487d6d2c70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<contours>)))
  "Returns full string definition for message of type '<contours>"
  (cl:format cl:nil "img_pro_info pro_info~%================================================================================~%MSG: opencvtest/img_pro_info~%    bool find_obs_flag~%    float64 dis~%    int32 pos_left~%    int32 pos_right~%    int32 x_pos~%    int32 y_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'contours)))
  "Returns full string definition for message of type 'contours"
  (cl:format cl:nil "img_pro_info pro_info~%================================================================================~%MSG: opencvtest/img_pro_info~%    bool find_obs_flag~%    float64 dis~%    int32 pos_left~%    int32 pos_right~%    int32 x_pos~%    int32 y_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <contours>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pro_info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <contours>))
  "Converts a ROS message object to a list"
  (cl:list 'contours
    (cl:cons ':pro_info (pro_info msg))
))
