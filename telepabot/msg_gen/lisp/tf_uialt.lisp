; Auto-generated. Do not edit!


(cl:in-package ui_alt-msg)


;//! \htmlinclude tf_uialt.msg.html

(cl:defclass <tf_uialt> (roslisp-msg-protocol:ros-message)
  ((src
    :reader src
    :initarg :src
    :type (cl:vector ui_alt-msg:src_tf_uialt)
   :initform (cl:make-array 0 :element-type 'ui_alt-msg:src_tf_uialt :initial-element (cl:make-instance 'ui_alt-msg:src_tf_uialt))))
)

(cl:defclass tf_uialt (<tf_uialt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tf_uialt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tf_uialt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ui_alt-msg:<tf_uialt> is deprecated: use ui_alt-msg:tf_uialt instead.")))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <tf_uialt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ui_alt-msg:src-val is deprecated.  Use ui_alt-msg:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tf_uialt>) ostream)
  "Serializes a message object of type '<tf_uialt>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'src))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tf_uialt>) istream)
  "Deserializes a message object of type '<tf_uialt>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ui_alt-msg:src_tf_uialt))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tf_uialt>)))
  "Returns string type for a message object of type '<tf_uialt>"
  "ui_alt/tf_uialt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tf_uialt)))
  "Returns string type for a message object of type 'tf_uialt"
  "ui_alt/tf_uialt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tf_uialt>)))
  "Returns md5sum for a message object of type '<tf_uialt>"
  "7ffcaccd814bf1cfa9bad231e0d4951e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tf_uialt)))
  "Returns md5sum for a message object of type 'tf_uialt"
  "7ffcaccd814bf1cfa9bad231e0d4951e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tf_uialt>)))
  "Returns full string definition for message of type '<tf_uialt>"
  (cl:format cl:nil "src_tf_uialt[] src~%================================================================================~%MSG: ui_alt/src_tf_uialt~%string parts~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tf_uialt)))
  "Returns full string definition for message of type 'tf_uialt"
  (cl:format cl:nil "src_tf_uialt[] src~%================================================================================~%MSG: ui_alt/src_tf_uialt~%string parts~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tf_uialt>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tf_uialt>))
  "Converts a ROS message object to a list"
  (cl:list 'tf_uialt
    (cl:cons ':src (src msg))
))
