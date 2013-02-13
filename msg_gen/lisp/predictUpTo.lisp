; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude predictUpTo.msg.html

(cl:defclass <predictUpTo> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:integer
    :initform 0)
   (seq_rpy
    :reader seq_rpy
    :initarg :seq_rpy
    :type cl:integer
    :initform 0)
   (seq_xyz
    :reader seq_xyz
    :initarg :seq_xyz
    :type cl:integer
    :initform 0)
   (controlInfo
    :reader controlInfo
    :initarg :controlInfo
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (altd
    :reader altd
    :initarg :altd
    :type cl:float
    :initform 0.0))
)

(cl:defclass predictUpTo (<predictUpTo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <predictUpTo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'predictUpTo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<predictUpTo> is deprecated: use AutoNav-msg:predictUpTo instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:timestamp-val is deprecated.  Use AutoNav-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'seq_rpy-val :lambda-list '(m))
(cl:defmethod seq_rpy-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:seq_rpy-val is deprecated.  Use AutoNav-msg:seq_rpy instead.")
  (seq_rpy m))

(cl:ensure-generic-function 'seq_xyz-val :lambda-list '(m))
(cl:defmethod seq_xyz-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:seq_xyz-val is deprecated.  Use AutoNav-msg:seq_xyz instead.")
  (seq_xyz m))

(cl:ensure-generic-function 'controlInfo-val :lambda-list '(m))
(cl:defmethod controlInfo-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:controlInfo-val is deprecated.  Use AutoNav-msg:controlInfo instead.")
  (controlInfo m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll-val is deprecated.  Use AutoNav-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch-val is deprecated.  Use AutoNav-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw-val is deprecated.  Use AutoNav-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vx-val is deprecated.  Use AutoNav-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vy-val is deprecated.  Use AutoNav-msg:vy instead.")
  (vy m))

(cl:ensure-generic-function 'altd-val :lambda-list '(m))
(cl:defmethod altd-val ((m <predictUpTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:altd-val is deprecated.  Use AutoNav-msg:altd instead.")
  (altd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <predictUpTo>) ostream)
  "Serializes a message object of type '<predictUpTo>"
  (cl:let* ((signed (cl:slot-value msg 'timestamp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq_rpy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq_rpy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq_rpy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq_rpy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq_xyz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq_xyz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq_xyz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq_xyz)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'controlInfo) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <predictUpTo>) istream)
  "Deserializes a message object of type '<predictUpTo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq_rpy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq_rpy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq_rpy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq_rpy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq_xyz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq_xyz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq_xyz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq_xyz)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'controlInfo) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<predictUpTo>)))
  "Returns string type for a message object of type '<predictUpTo>"
  "AutoNav/predictUpTo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'predictUpTo)))
  "Returns string type for a message object of type 'predictUpTo"
  "AutoNav/predictUpTo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<predictUpTo>)))
  "Returns md5sum for a message object of type '<predictUpTo>"
  "dd021969d994e39ff43a715cb3a3d6ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'predictUpTo)))
  "Returns md5sum for a message object of type 'predictUpTo"
  "dd021969d994e39ff43a715cb3a3d6ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<predictUpTo>)))
  "Returns full string definition for message of type '<predictUpTo>"
  (cl:format cl:nil "int32 timestamp~%uint32 seq_rpy~%uint32 seq_xyz~%~%geometry_msgs/Twist controlInfo~%~%float32 roll~%float32 pitch~%float32 yaw~%~%float32 vx~%float32 vy~%float32 altd~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'predictUpTo)))
  "Returns full string definition for message of type 'predictUpTo"
  (cl:format cl:nil "int32 timestamp~%uint32 seq_rpy~%uint32 seq_xyz~%~%geometry_msgs/Twist controlInfo~%~%float32 roll~%float32 pitch~%float32 yaw~%~%float32 vx~%float32 vy~%float32 altd~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <predictUpTo>))
  (cl:+ 0
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'controlInfo))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <predictUpTo>))
  "Converts a ROS message object to a list"
  (cl:list 'predictUpTo
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':seq_rpy (seq_rpy msg))
    (cl:cons ':seq_xyz (seq_xyz msg))
    (cl:cons ':controlInfo (controlInfo msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':altd (altd msg))
))
