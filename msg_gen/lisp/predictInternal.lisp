; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude predictInternal.msg.html

(cl:defclass <predictInternal> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:integer
    :initform 0)
   (timeSpanMicros
    :reader timeSpanMicros
    :initarg :timeSpanMicros
    :type cl:integer
    :initform 0)
   (activeControlInfo
    :reader activeControlInfo
    :initarg :activeControlInfo
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (useControlGains
    :reader useControlGains
    :initarg :useControlGains
    :type cl:integer
    :initform 0)
   (controlValid
    :reader controlValid
    :initarg :controlValid
    :type cl:integer
    :initform 0)
   (rollControlGain
    :reader rollControlGain
    :initarg :rollControlGain
    :type cl:float
    :initform 0.0)
   (pitchControlGain
    :reader pitchControlGain
    :initarg :pitchControlGain
    :type cl:float
    :initform 0.0)
   (yawSpeedControlGain
    :reader yawSpeedControlGain
    :initarg :yawSpeedControlGain
    :type cl:float
    :initform 0.0)
   (forceX
    :reader forceX
    :initarg :forceX
    :type cl:float
    :initform 0.0)
   (forceY
    :reader forceY
    :initarg :forceY
    :type cl:float
    :initform 0.0)
   (vx_gain
    :reader vx_gain
    :initarg :vx_gain
    :type cl:float
    :initform 0.0)
   (vy_gain
    :reader vy_gain
    :initarg :vy_gain
    :type cl:float
    :initform 0.0)
   (vz_gain
    :reader vz_gain
    :initarg :vz_gain
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
   (x_pre
    :reader x_pre
    :initarg :x_pre
    :type cl:float
    :initform 0.0)
   (dx_pre
    :reader dx_pre
    :initarg :dx_pre
    :type cl:float
    :initform 0.0)
   (y_pre
    :reader y_pre
    :initarg :y_pre
    :type cl:float
    :initform 0.0)
   (dy_pre
    :reader dy_pre
    :initarg :dy_pre
    :type cl:float
    :initform 0.0)
   (z_pre
    :reader z_pre
    :initarg :z_pre
    :type cl:float
    :initform 0.0)
   (dz_pre
    :reader dz_pre
    :initarg :dz_pre
    :type cl:float
    :initform 0.0)
   (varx_pre
    :reader varx_pre
    :initarg :varx_pre
    :type cl:float
    :initform 0.0)
   (vary_pre
    :reader vary_pre
    :initarg :vary_pre
    :type cl:float
    :initform 0.0)
   (vardx_pre
    :reader vardx_pre
    :initarg :vardx_pre
    :type cl:float
    :initform 0.0)
   (vardy_pre
    :reader vardy_pre
    :initarg :vardy_pre
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
    :initform 0.0)
   (x_post
    :reader x_post
    :initarg :x_post
    :type cl:float
    :initform 0.0)
   (dx_post
    :reader dx_post
    :initarg :dx_post
    :type cl:float
    :initform 0.0)
   (y_post
    :reader y_post
    :initarg :y_post
    :type cl:float
    :initform 0.0)
   (dy_post
    :reader dy_post
    :initarg :dy_post
    :type cl:float
    :initform 0.0)
   (z_post
    :reader z_post
    :initarg :z_post
    :type cl:float
    :initform 0.0)
   (dz_post
    :reader dz_post
    :initarg :dz_post
    :type cl:float
    :initform 0.0)
   (varx_post
    :reader varx_post
    :initarg :varx_post
    :type cl:float
    :initform 0.0)
   (vary_post
    :reader vary_post
    :initarg :vary_post
    :type cl:float
    :initform 0.0)
   (vardx_post
    :reader vardx_post
    :initarg :vardx_post
    :type cl:float
    :initform 0.0)
   (vardy_post
    :reader vardy_post
    :initarg :vardy_post
    :type cl:float
    :initform 0.0))
)

(cl:defclass predictInternal (<predictInternal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <predictInternal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'predictInternal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<predictInternal> is deprecated: use AutoNav-msg:predictInternal instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:timestamp-val is deprecated.  Use AutoNav-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'timeSpanMicros-val :lambda-list '(m))
(cl:defmethod timeSpanMicros-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:timeSpanMicros-val is deprecated.  Use AutoNav-msg:timeSpanMicros instead.")
  (timeSpanMicros m))

(cl:ensure-generic-function 'activeControlInfo-val :lambda-list '(m))
(cl:defmethod activeControlInfo-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:activeControlInfo-val is deprecated.  Use AutoNav-msg:activeControlInfo instead.")
  (activeControlInfo m))

(cl:ensure-generic-function 'useControlGains-val :lambda-list '(m))
(cl:defmethod useControlGains-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:useControlGains-val is deprecated.  Use AutoNav-msg:useControlGains instead.")
  (useControlGains m))

(cl:ensure-generic-function 'controlValid-val :lambda-list '(m))
(cl:defmethod controlValid-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:controlValid-val is deprecated.  Use AutoNav-msg:controlValid instead.")
  (controlValid m))

(cl:ensure-generic-function 'rollControlGain-val :lambda-list '(m))
(cl:defmethod rollControlGain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:rollControlGain-val is deprecated.  Use AutoNav-msg:rollControlGain instead.")
  (rollControlGain m))

(cl:ensure-generic-function 'pitchControlGain-val :lambda-list '(m))
(cl:defmethod pitchControlGain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitchControlGain-val is deprecated.  Use AutoNav-msg:pitchControlGain instead.")
  (pitchControlGain m))

(cl:ensure-generic-function 'yawSpeedControlGain-val :lambda-list '(m))
(cl:defmethod yawSpeedControlGain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yawSpeedControlGain-val is deprecated.  Use AutoNav-msg:yawSpeedControlGain instead.")
  (yawSpeedControlGain m))

(cl:ensure-generic-function 'forceX-val :lambda-list '(m))
(cl:defmethod forceX-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:forceX-val is deprecated.  Use AutoNav-msg:forceX instead.")
  (forceX m))

(cl:ensure-generic-function 'forceY-val :lambda-list '(m))
(cl:defmethod forceY-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:forceY-val is deprecated.  Use AutoNav-msg:forceY instead.")
  (forceY m))

(cl:ensure-generic-function 'vx_gain-val :lambda-list '(m))
(cl:defmethod vx_gain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vx_gain-val is deprecated.  Use AutoNav-msg:vx_gain instead.")
  (vx_gain m))

(cl:ensure-generic-function 'vy_gain-val :lambda-list '(m))
(cl:defmethod vy_gain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vy_gain-val is deprecated.  Use AutoNav-msg:vy_gain instead.")
  (vy_gain m))

(cl:ensure-generic-function 'vz_gain-val :lambda-list '(m))
(cl:defmethod vz_gain-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vz_gain-val is deprecated.  Use AutoNav-msg:vz_gain instead.")
  (vz_gain m))

(cl:ensure-generic-function 'roll_pre-val :lambda-list '(m))
(cl:defmethod roll_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll_pre-val is deprecated.  Use AutoNav-msg:roll_pre instead.")
  (roll_pre m))

(cl:ensure-generic-function 'pitch_pre-val :lambda-list '(m))
(cl:defmethod pitch_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch_pre-val is deprecated.  Use AutoNav-msg:pitch_pre instead.")
  (pitch_pre m))

(cl:ensure-generic-function 'yaw_pre-val :lambda-list '(m))
(cl:defmethod yaw_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw_pre-val is deprecated.  Use AutoNav-msg:yaw_pre instead.")
  (yaw_pre m))

(cl:ensure-generic-function 'dyaw_pre-val :lambda-list '(m))
(cl:defmethod dyaw_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dyaw_pre-val is deprecated.  Use AutoNav-msg:dyaw_pre instead.")
  (dyaw_pre m))

(cl:ensure-generic-function 'x_pre-val :lambda-list '(m))
(cl:defmethod x_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:x_pre-val is deprecated.  Use AutoNav-msg:x_pre instead.")
  (x_pre m))

(cl:ensure-generic-function 'dx_pre-val :lambda-list '(m))
(cl:defmethod dx_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dx_pre-val is deprecated.  Use AutoNav-msg:dx_pre instead.")
  (dx_pre m))

(cl:ensure-generic-function 'y_pre-val :lambda-list '(m))
(cl:defmethod y_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:y_pre-val is deprecated.  Use AutoNav-msg:y_pre instead.")
  (y_pre m))

(cl:ensure-generic-function 'dy_pre-val :lambda-list '(m))
(cl:defmethod dy_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dy_pre-val is deprecated.  Use AutoNav-msg:dy_pre instead.")
  (dy_pre m))

(cl:ensure-generic-function 'z_pre-val :lambda-list '(m))
(cl:defmethod z_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:z_pre-val is deprecated.  Use AutoNav-msg:z_pre instead.")
  (z_pre m))

(cl:ensure-generic-function 'dz_pre-val :lambda-list '(m))
(cl:defmethod dz_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dz_pre-val is deprecated.  Use AutoNav-msg:dz_pre instead.")
  (dz_pre m))

(cl:ensure-generic-function 'varx_pre-val :lambda-list '(m))
(cl:defmethod varx_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:varx_pre-val is deprecated.  Use AutoNav-msg:varx_pre instead.")
  (varx_pre m))

(cl:ensure-generic-function 'vary_pre-val :lambda-list '(m))
(cl:defmethod vary_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vary_pre-val is deprecated.  Use AutoNav-msg:vary_pre instead.")
  (vary_pre m))

(cl:ensure-generic-function 'vardx_pre-val :lambda-list '(m))
(cl:defmethod vardx_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vardx_pre-val is deprecated.  Use AutoNav-msg:vardx_pre instead.")
  (vardx_pre m))

(cl:ensure-generic-function 'vardy_pre-val :lambda-list '(m))
(cl:defmethod vardy_pre-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vardy_pre-val is deprecated.  Use AutoNav-msg:vardy_pre instead.")
  (vardy_pre m))

(cl:ensure-generic-function 'roll_post-val :lambda-list '(m))
(cl:defmethod roll_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:roll_post-val is deprecated.  Use AutoNav-msg:roll_post instead.")
  (roll_post m))

(cl:ensure-generic-function 'pitch_post-val :lambda-list '(m))
(cl:defmethod pitch_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pitch_post-val is deprecated.  Use AutoNav-msg:pitch_post instead.")
  (pitch_post m))

(cl:ensure-generic-function 'yaw_post-val :lambda-list '(m))
(cl:defmethod yaw_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw_post-val is deprecated.  Use AutoNav-msg:yaw_post instead.")
  (yaw_post m))

(cl:ensure-generic-function 'dyaw_post-val :lambda-list '(m))
(cl:defmethod dyaw_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dyaw_post-val is deprecated.  Use AutoNav-msg:dyaw_post instead.")
  (dyaw_post m))

(cl:ensure-generic-function 'x_post-val :lambda-list '(m))
(cl:defmethod x_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:x_post-val is deprecated.  Use AutoNav-msg:x_post instead.")
  (x_post m))

(cl:ensure-generic-function 'dx_post-val :lambda-list '(m))
(cl:defmethod dx_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dx_post-val is deprecated.  Use AutoNav-msg:dx_post instead.")
  (dx_post m))

(cl:ensure-generic-function 'y_post-val :lambda-list '(m))
(cl:defmethod y_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:y_post-val is deprecated.  Use AutoNav-msg:y_post instead.")
  (y_post m))

(cl:ensure-generic-function 'dy_post-val :lambda-list '(m))
(cl:defmethod dy_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dy_post-val is deprecated.  Use AutoNav-msg:dy_post instead.")
  (dy_post m))

(cl:ensure-generic-function 'z_post-val :lambda-list '(m))
(cl:defmethod z_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:z_post-val is deprecated.  Use AutoNav-msg:z_post instead.")
  (z_post m))

(cl:ensure-generic-function 'dz_post-val :lambda-list '(m))
(cl:defmethod dz_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dz_post-val is deprecated.  Use AutoNav-msg:dz_post instead.")
  (dz_post m))

(cl:ensure-generic-function 'varx_post-val :lambda-list '(m))
(cl:defmethod varx_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:varx_post-val is deprecated.  Use AutoNav-msg:varx_post instead.")
  (varx_post m))

(cl:ensure-generic-function 'vary_post-val :lambda-list '(m))
(cl:defmethod vary_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vary_post-val is deprecated.  Use AutoNav-msg:vary_post instead.")
  (vary_post m))

(cl:ensure-generic-function 'vardx_post-val :lambda-list '(m))
(cl:defmethod vardx_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vardx_post-val is deprecated.  Use AutoNav-msg:vardx_post instead.")
  (vardx_post m))

(cl:ensure-generic-function 'vardy_post-val :lambda-list '(m))
(cl:defmethod vardy_post-val ((m <predictInternal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vardy_post-val is deprecated.  Use AutoNav-msg:vardy_post instead.")
  (vardy_post m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <predictInternal>) ostream)
  "Serializes a message object of type '<predictInternal>"
  (cl:let* ((signed (cl:slot-value msg 'timestamp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'timeSpanMicros)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'activeControlInfo) ostream)
  (cl:let* ((signed (cl:slot-value msg 'useControlGains)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'controlValid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rollControlGain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitchControlGain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yawSpeedControlGain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forceX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forceY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vz_gain))))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dx_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dy_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dz_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'varx_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vary_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vardx_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vardy_pre))))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dx_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dy_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dz_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'varx_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vary_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vardx_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vardy_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <predictInternal>) istream)
  "Deserializes a message object of type '<predictInternal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeSpanMicros) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'activeControlInfo) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'useControlGains) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controlValid) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rollControlGain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitchControlGain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yawSpeedControlGain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forceX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forceY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vz_gain) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'x_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dx_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dy_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dz_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'varx_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vary_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vardx_pre) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vardy_pre) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dx_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dy_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dz_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'varx_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vary_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vardx_post) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vardy_post) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<predictInternal>)))
  "Returns string type for a message object of type '<predictInternal>"
  "AutoNav/predictInternal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'predictInternal)))
  "Returns string type for a message object of type 'predictInternal"
  "AutoNav/predictInternal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<predictInternal>)))
  "Returns md5sum for a message object of type '<predictInternal>"
  "ecda137da737bf0c26f0e9aeaeab8110")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'predictInternal)))
  "Returns md5sum for a message object of type 'predictInternal"
  "ecda137da737bf0c26f0e9aeaeab8110")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<predictInternal>)))
  "Returns full string definition for message of type '<predictInternal>"
  (cl:format cl:nil "#time when message was published~%int32 timestamp~%~%int32 timeSpanMicros~%~%geometry_msgs/Twist activeControlInfo~%~%int32 useControlGains~%int32 controlValid~%~%# gains because of given Control Commands~%float32 rollControlGain~%float32 pitchControlGain~%float32 yawSpeedControlGain~%~%#Forces project on horizontal axes due to tilt~%float32 forceX~%float32 forceY~%~%#Velocity gain due to forces on horizontal axes~%float32 vx_gain~%float32 vy_gain~%float32 vz_gain~%~%#prior state before prediction~%float32 roll_pre~%float32 pitch_pre~%float32 yaw_pre~%float32 dyaw_pre~%float32 x_pre~%float32 dx_pre~%float32 y_pre~%float32 dy_pre~%float32 z_pre~%float32 dz_pre~%float32 varx_pre~%float32 vary_pre~%float32 vardx_pre~%float32 vardy_pre~%~%#posterior state after prediction~%float32 roll_post~%float32 pitch_post~%float32 yaw_post~%float32 dyaw_post~%float32 x_post~%float32 dx_post~%float32 y_post~%float32 dy_post~%float32 z_post~%float32 dz_post~%float32 varx_post~%float32 vary_post~%float32 vardx_post~%float32 vardy_post~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'predictInternal)))
  "Returns full string definition for message of type 'predictInternal"
  (cl:format cl:nil "#time when message was published~%int32 timestamp~%~%int32 timeSpanMicros~%~%geometry_msgs/Twist activeControlInfo~%~%int32 useControlGains~%int32 controlValid~%~%# gains because of given Control Commands~%float32 rollControlGain~%float32 pitchControlGain~%float32 yawSpeedControlGain~%~%#Forces project on horizontal axes due to tilt~%float32 forceX~%float32 forceY~%~%#Velocity gain due to forces on horizontal axes~%float32 vx_gain~%float32 vy_gain~%float32 vz_gain~%~%#prior state before prediction~%float32 roll_pre~%float32 pitch_pre~%float32 yaw_pre~%float32 dyaw_pre~%float32 x_pre~%float32 dx_pre~%float32 y_pre~%float32 dy_pre~%float32 z_pre~%float32 dz_pre~%float32 varx_pre~%float32 vary_pre~%float32 vardx_pre~%float32 vardy_pre~%~%#posterior state after prediction~%float32 roll_post~%float32 pitch_post~%float32 yaw_post~%float32 dyaw_post~%float32 x_post~%float32 dx_post~%float32 y_post~%float32 dy_post~%float32 z_post~%float32 dz_post~%float32 varx_post~%float32 vary_post~%float32 vardx_post~%float32 vardy_post~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <predictInternal>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'activeControlInfo))
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
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <predictInternal>))
  "Converts a ROS message object to a list"
  (cl:list 'predictInternal
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':timeSpanMicros (timeSpanMicros msg))
    (cl:cons ':activeControlInfo (activeControlInfo msg))
    (cl:cons ':useControlGains (useControlGains msg))
    (cl:cons ':controlValid (controlValid msg))
    (cl:cons ':rollControlGain (rollControlGain msg))
    (cl:cons ':pitchControlGain (pitchControlGain msg))
    (cl:cons ':yawSpeedControlGain (yawSpeedControlGain msg))
    (cl:cons ':forceX (forceX msg))
    (cl:cons ':forceY (forceY msg))
    (cl:cons ':vx_gain (vx_gain msg))
    (cl:cons ':vy_gain (vy_gain msg))
    (cl:cons ':vz_gain (vz_gain msg))
    (cl:cons ':roll_pre (roll_pre msg))
    (cl:cons ':pitch_pre (pitch_pre msg))
    (cl:cons ':yaw_pre (yaw_pre msg))
    (cl:cons ':dyaw_pre (dyaw_pre msg))
    (cl:cons ':x_pre (x_pre msg))
    (cl:cons ':dx_pre (dx_pre msg))
    (cl:cons ':y_pre (y_pre msg))
    (cl:cons ':dy_pre (dy_pre msg))
    (cl:cons ':z_pre (z_pre msg))
    (cl:cons ':dz_pre (dz_pre msg))
    (cl:cons ':varx_pre (varx_pre msg))
    (cl:cons ':vary_pre (vary_pre msg))
    (cl:cons ':vardx_pre (vardx_pre msg))
    (cl:cons ':vardy_pre (vardy_pre msg))
    (cl:cons ':roll_post (roll_post msg))
    (cl:cons ':pitch_post (pitch_post msg))
    (cl:cons ':yaw_post (yaw_post msg))
    (cl:cons ':dyaw_post (dyaw_post msg))
    (cl:cons ':x_post (x_post msg))
    (cl:cons ':dx_post (dx_post msg))
    (cl:cons ':y_post (y_post msg))
    (cl:cons ':dy_post (dy_post msg))
    (cl:cons ':z_post (z_post msg))
    (cl:cons ':dz_post (dz_post msg))
    (cl:cons ':varx_post (varx_post msg))
    (cl:cons ':vary_post (vary_post msg))
    (cl:cons ':vardx_post (vardx_post msg))
    (cl:cons ':vardy_post (vardy_post msg))
))
