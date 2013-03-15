; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude obs_IMU_RPY.msg.html

(cl:defclass <obs_IMU_RPY> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:integer
    :initform 0)
   (seq
    :reader seq
    :initarg :seq
    :type cl:integer
    :initform 0)
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
   (baselineY_IMU
    :reader baselineY_IMU
    :initarg :baselineY_IMU
    :type cl:float
    :initform 0.0)
   (baselineY_Filter
    :reader baselineY_Filter
    :initarg :baselineY_Filter
    :type cl:float
    :initform 0.0)
   (navYaw
    :reader navYaw
    :initarg :navYaw
    :type cl:float
    :initform 0.0)
   (observedYaw
    :reader observedYaw
    :initarg :observedYaw
    :type cl:float
    :initform 0.0)
   (roll_pre
    :reader roll_pre
    :initarg :roll_pre
    :type cl:float
    :initform 0.0)
   (pitch_pre
    :reader pitch_pre
    :initarg :pitch_pre
    :type cl:float
    :initform 0.0)
   (yaw_pre
    :reader yaw_pre
    :initarg :yaw_pre
    :type cl:float
    :initform 0.0)
   (dyaw_pre
    :reader dyaw_pre
    :initarg :dyaw_pre
    :type cl:float
    :initform 0.0)
   (roll_post
    :reader roll_post
    :initarg :roll_post
    :type cl:float
    :initform 0.0)
   (pitch_post
    :reader pitch_post
    :initarg :pitch_post
    :type cl:float
    :initform 0.0)
   (yaw_post
    :reader yaw_post
    :initarg :yaw_post
    :type cl:float
    :initform 0.0)
   (dyaw_post
    :reader dyaw_post
    :initarg :dyaw_post
    :type cl:float
    :initform 0.0))
)

(cl:defclass obs_IMU_RPY (<obs_IMU_RPY>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obs_IMU_RPY>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obs_IMU_RPY)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<obs_IMU_RPY> is deprecated: use AutoNav-msg:obs_IMU_RPY instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:timestamp-val is deprecated.  Use AutoNav-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:seq-val is deprecated.  Use AutoNav-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll-val is deprecated.  Use AutoNav-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch-val is deprecated.  Use AutoNav-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'baselineY_IMU-val :lambda-list '(m))
(cl:defmethod baselineY_IMU-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:baselineY_IMU-val is deprecated.  Use AutoNav-msg:baselineY_IMU instead.")
  (baselineY_IMU m))

(cl:ensure-generic-function 'baselineY_Filter-val :lambda-list '(m))
(cl:defmethod baselineY_Filter-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:baselineY_Filter-val is deprecated.  Use AutoNav-msg:baselineY_Filter instead.")
  (baselineY_Filter m))

(cl:ensure-generic-function 'navYaw-val :lambda-list '(m))
(cl:defmethod navYaw-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:navYaw-val is deprecated.  Use AutoNav-msg:navYaw instead.")
  (navYaw m))

(cl:ensure-generic-function 'observedYaw-val :lambda-list '(m))
(cl:defmethod observedYaw-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:observedYaw-val is deprecated.  Use AutoNav-msg:observedYaw instead.")
  (observedYaw m))

(cl:ensure-generic-function 'roll_pre-val :lambda-list '(m))
(cl:defmethod roll_pre-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll_pre-val is deprecated.  Use AutoNav-msg:roll_pre instead.")
  (roll_pre m))

(cl:ensure-generic-function 'pitch_pre-val :lambda-list '(m))
(cl:defmethod pitch_pre-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch_pre-val is deprecated.  Use AutoNav-msg:pitch_pre instead.")
  (pitch_pre m))

(cl:ensure-generic-function 'yaw_pre-val :lambda-list '(m))
(cl:defmethod yaw_pre-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw_pre-val is deprecated.  Use AutoNav-msg:yaw_pre instead.")
  (yaw_pre m))

(cl:ensure-generic-function 'dyaw_pre-val :lambda-list '(m))
(cl:defmethod dyaw_pre-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dyaw_pre-val is deprecated.  Use AutoNav-msg:dyaw_pre instead.")
  (dyaw_pre m))

(cl:ensure-generic-function 'roll_post-val :lambda-list '(m))
(cl:defmethod roll_post-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll_post-val is deprecated.  Use AutoNav-msg:roll_post instead.")
  (roll_post m))

(cl:ensure-generic-function 'pitch_post-val :lambda-list '(m))
(cl:defmethod pitch_post-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch_post-val is deprecated.  Use AutoNav-msg:pitch_post instead.")
  (pitch_post m))

(cl:ensure-generic-function 'yaw_post-val :lambda-list '(m))
(cl:defmethod yaw_post-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw_post-val is deprecated.  Use AutoNav-msg:yaw_post instead.")
  (yaw_post m))

(cl:ensure-generic-function 'dyaw_post-val :lambda-list '(m))
(cl:defmethod dyaw_post-val ((m <obs_IMU_RPY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dyaw_post-val is deprecated.  Use AutoNav-msg:dyaw_post instead.")
  (dyaw_post m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obs_IMU_RPY>) ostream)
  "Serializes a message object of type '<obs_IMU_RPY>"
  (cl:let* ((signed (cl:slot-value msg 'timestamp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'baselineY_IMU))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'baselineY_Filter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'navYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'observedYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dyaw_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dyaw_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obs_IMU_RPY>) istream)
  "Deserializes a message object of type '<obs_IMU_RPY>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) (cl:read-byte istream))
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
    (cl:setf (cl:slot-value msg 'baselineY_IMU) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'baselineY_Filter) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'navYaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'observedYaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dyaw_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dyaw_post) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obs_IMU_RPY>)))
  "Returns string type for a message object of type '<obs_IMU_RPY>"
  "AutoNav/obs_IMU_RPY")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obs_IMU_RPY)))
  "Returns string type for a message object of type 'obs_IMU_RPY"
  "AutoNav/obs_IMU_RPY")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obs_IMU_RPY>)))
  "Returns md5sum for a message object of type '<obs_IMU_RPY>"
  "a9a4538ac5c69253cc8a1aefb110a3fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obs_IMU_RPY)))
  "Returns md5sum for a message object of type 'obs_IMU_RPY"
  "a9a4538ac5c69253cc8a1aefb110a3fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obs_IMU_RPY>)))
  "Returns full string definition for message of type '<obs_IMU_RPY>"
  (cl:format cl:nil "int32 timestamp~%uint32 seq~%~%float32 roll~%float32 pitch~%~%float32 baselineY_IMU~%float32 baselineY_Filter~%float32 navYaw~%float32 observedYaw~%~%float32 roll_pre~%float32 pitch_pre~%float32 yaw_pre~%float32 dyaw_pre~%~%float32 roll_post~%float32 pitch_post~%float32 yaw_post~%float32 dyaw_post~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obs_IMU_RPY)))
  "Returns full string definition for message of type 'obs_IMU_RPY"
  (cl:format cl:nil "int32 timestamp~%uint32 seq~%~%float32 roll~%float32 pitch~%~%float32 baselineY_IMU~%float32 baselineY_Filter~%float32 navYaw~%float32 observedYaw~%~%float32 roll_pre~%float32 pitch_pre~%float32 yaw_pre~%float32 dyaw_pre~%~%float32 roll_post~%float32 pitch_post~%float32 yaw_post~%float32 dyaw_post~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obs_IMU_RPY>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obs_IMU_RPY>))
  "Converts a ROS message object to a list"
  (cl:list 'obs_IMU_RPY
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':seq (seq msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':baselineY_IMU (baselineY_IMU msg))
    (cl:cons ':baselineY_Filter (baselineY_Filter msg))
    (cl:cons ':navYaw (navYaw msg))
    (cl:cons ':observedYaw (observedYaw msg))
    (cl:cons ':roll_pre (roll_pre msg))
    (cl:cons ':pitch_pre (pitch_pre msg))
    (cl:cons ':yaw_pre (yaw_pre msg))
    (cl:cons ':dyaw_pre (dyaw_pre msg))
    (cl:cons ':roll_post (roll_post msg))
    (cl:cons ':pitch_post (pitch_post msg))
    (cl:cons ':yaw_post (yaw_post msg))
    (cl:cons ':dyaw_post (dyaw_post msg))
))
