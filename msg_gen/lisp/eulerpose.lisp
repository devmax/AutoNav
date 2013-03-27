; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude eulerpose.msg.html

(cl:defclass <eulerpose> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (euler
    :reader euler
    :initarg :euler
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass eulerpose (<eulerpose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eulerpose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eulerpose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<eulerpose> is deprecated: use AutoNav-msg:eulerpose instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <eulerpose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:position-val is deprecated.  Use AutoNav-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'euler-val :lambda-list '(m))
(cl:defmethod euler-val ((m <eulerpose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:euler-val is deprecated.  Use AutoNav-msg:euler instead.")
  (euler m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eulerpose>) ostream)
  "Serializes a message object of type '<eulerpose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'euler) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eulerpose>) istream)
  "Deserializes a message object of type '<eulerpose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'euler) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eulerpose>)))
  "Returns string type for a message object of type '<eulerpose>"
  "AutoNav/eulerpose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eulerpose)))
  "Returns string type for a message object of type 'eulerpose"
  "AutoNav/eulerpose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eulerpose>)))
  "Returns md5sum for a message object of type '<eulerpose>"
  "49488e00b80848fd7fea61173d6d3db2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eulerpose)))
  "Returns md5sum for a message object of type 'eulerpose"
  "49488e00b80848fd7fea61173d6d3db2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eulerpose>)))
  "Returns full string definition for message of type '<eulerpose>"
  (cl:format cl:nil "geometry_msgs/Vector3 position~%~%geometry_msgs/Vector3 euler~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eulerpose)))
  "Returns full string definition for message of type 'eulerpose"
  (cl:format cl:nil "geometry_msgs/Vector3 position~%~%geometry_msgs/Vector3 euler~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eulerpose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'euler))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eulerpose>))
  "Converts a ROS message object to a list"
  (cl:list 'eulerpose
    (cl:cons ':position (position msg))
    (cl:cons ':euler (euler msg))
))
