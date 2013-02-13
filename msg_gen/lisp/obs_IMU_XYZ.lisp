; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude obs_IMU_XYZ.msg.html

(cl:defclass <obs_IMU_XYZ> (roslisp-msg-protocol:ros-message)
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
   (nav_vx
    :reader nav_vx
    :initarg :nav_vx
    :type cl:float
    :initform 0.0)
   (nav_vy
    :reader nav_vy
    :initarg :nav_vy
    :type cl:float
    :initform 0.0)
   (global_vx
    :reader global_vx
    :initarg :global_vx
    :type cl:float
    :initform 0.0)
   (global_vy
    :reader global_vy
    :initarg :global_vy
    :type cl:float
    :initform 0.0)
   (x_pre
    :reader x_pre
    :initarg :x_pre
    :type cl:float
    :initform 0.0)
   (y_pre
    :reader y_pre
    :initarg :y_pre
    :type cl:float
    :initform 0.0)
   (z_pre
    :reader z_pre
    :initarg :z_pre
    :type cl:float
    :initform 0.0)
   (dx_pre
    :reader dx_pre
    :initarg :dx_pre
    :type cl:float
    :initform 0.0)
   (dy_pre
    :reader dy_pre
    :initarg :dy_pre
    :type cl:float
    :initform 0.0)
   (dz_pre
    :reader dz_pre
    :initarg :dz_pre
    :type cl:float
    :initform 0.0)
   (x_post
    :reader x_post
    :initarg :x_post
    :type cl:float
    :initform 0.0)
   (y_post
    :reader y_post
    :initarg :y_post
    :type cl:float
    :initform 0.0)
   (z_post
    :reader z_post
    :initarg :z_post
    :type cl:float
    :initform 0.0)
   (dx_post
    :reader dx_post
    :initarg :dx_post
    :type cl:float
    :initform 0.0)
   (dy_post
    :reader dy_post
    :initarg :dy_post
    :type cl:float
    :initform 0.0)
   (dz_post
    :reader dz_post
    :initarg :dz_post
    :type cl:float
    :initform 0.0))
)

(cl:defclass obs_IMU_XYZ (<obs_IMU_XYZ>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obs_IMU_XYZ>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obs_IMU_XYZ)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<obs_IMU_XYZ> is deprecated: use AutoNav-msg:obs_IMU_XYZ instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:timestamp-val is deprecated.  Use AutoNav-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:seq-val is deprecated.  Use AutoNav-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'nav_vx-val :lambda-list '(m))
(cl:defmethod nav_vx-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:nav_vx-val is deprecated.  Use AutoNav-msg:nav_vx instead.")
  (nav_vx m))

(cl:ensure-generic-function 'nav_vy-val :lambda-list '(m))
(cl:defmethod nav_vy-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:nav_vy-val is deprecated.  Use AutoNav-msg:nav_vy instead.")
  (nav_vy m))

(cl:ensure-generic-function 'global_vx-val :lambda-list '(m))
(cl:defmethod global_vx-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:global_vx-val is deprecated.  Use AutoNav-msg:global_vx instead.")
  (global_vx m))

(cl:ensure-generic-function 'global_vy-val :lambda-list '(m))
(cl:defmethod global_vy-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:global_vy-val is deprecated.  Use AutoNav-msg:global_vy instead.")
  (global_vy m))

(cl:ensure-generic-function 'x_pre-val :lambda-list '(m))
(cl:defmethod x_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:x_pre-val is deprecated.  Use AutoNav-msg:x_pre instead.")
  (x_pre m))

(cl:ensure-generic-function 'y_pre-val :lambda-list '(m))
(cl:defmethod y_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:y_pre-val is deprecated.  Use AutoNav-msg:y_pre instead.")
  (y_pre m))

(cl:ensure-generic-function 'z_pre-val :lambda-list '(m))
(cl:defmethod z_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:z_pre-val is deprecated.  Use AutoNav-msg:z_pre instead.")
  (z_pre m))

(cl:ensure-generic-function 'dx_pre-val :lambda-list '(m))
(cl:defmethod dx_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dx_pre-val is deprecated.  Use AutoNav-msg:dx_pre instead.")
  (dx_pre m))

(cl:ensure-generic-function 'dy_pre-val :lambda-list '(m))
(cl:defmethod dy_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dy_pre-val is deprecated.  Use AutoNav-msg:dy_pre instead.")
  (dy_pre m))

(cl:ensure-generic-function 'dz_pre-val :lambda-list '(m))
(cl:defmethod dz_pre-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dz_pre-val is deprecated.  Use AutoNav-msg:dz_pre instead.")
  (dz_pre m))

(cl:ensure-generic-function 'x_post-val :lambda-list '(m))
(cl:defmethod x_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:x_post-val is deprecated.  Use AutoNav-msg:x_post instead.")
  (x_post m))

(cl:ensure-generic-function 'y_post-val :lambda-list '(m))
(cl:defmethod y_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:y_post-val is deprecated.  Use AutoNav-msg:y_post instead.")
  (y_post m))

(cl:ensure-generic-function 'z_post-val :lambda-list '(m))
(cl:defmethod z_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:z_post-val is deprecated.  Use AutoNav-msg:z_post instead.")
  (z_post m))

(cl:ensure-generic-function 'dx_post-val :lambda-list '(m))
(cl:defmethod dx_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dx_post-val is deprecated.  Use AutoNav-msg:dx_post instead.")
  (dx_post m))

(cl:ensure-generic-function 'dy_post-val :lambda-list '(m))
(cl:defmethod dy_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dy_post-val is deprecated.  Use AutoNav-msg:dy_post instead.")
  (dy_post m))

(cl:ensure-generic-function 'dz_post-val :lambda-list '(m))
(cl:defmethod dz_post-val ((m <obs_IMU_XYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dz_post-val is deprecated.  Use AutoNav-msg:dz_post instead.")
  (dz_post m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obs_IMU_XYZ>) ostream)
  "Serializes a message object of type '<obs_IMU_XYZ>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nav_vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nav_vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'global_vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'global_vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dx_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dy_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dz_pre))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dx_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dy_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dz_post))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obs_IMU_XYZ>) istream)
  "Deserializes a message object of type '<obs_IMU_XYZ>"
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
    (cl:setf (cl:slot-value msg 'nav_vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nav_vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'global_vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'global_vy) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'y_pre) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'dx_pre) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'dz_pre) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'y_post) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'dx_post) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'dz_post) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obs_IMU_XYZ>)))
  "Returns string type for a message object of type '<obs_IMU_XYZ>"
  "AutoNav/obs_IMU_XYZ")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obs_IMU_XYZ)))
  "Returns string type for a message object of type 'obs_IMU_XYZ"
  "AutoNav/obs_IMU_XYZ")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obs_IMU_XYZ>)))
  "Returns md5sum for a message object of type '<obs_IMU_XYZ>"
  "1476010774524ca9679f455b1a546c2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obs_IMU_XYZ)))
  "Returns md5sum for a message object of type 'obs_IMU_XYZ"
  "1476010774524ca9679f455b1a546c2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obs_IMU_XYZ>)))
  "Returns full string definition for message of type '<obs_IMU_XYZ>"
  (cl:format cl:nil "int32 timestamp~%uint32 seq~%~%float32 nav_vx~%float32 nav_vy~%~%float32 global_vx~%float32 global_vy~%~%float32 x_pre~%float32 y_pre~%float32 z_pre~%~%float32 dx_pre~%float32 dy_pre~%float32 dz_pre~%~%float32 x_post~%float32 y_post~%float32 z_post~%~%float32 dx_post~%float32 dy_post~%float32 dz_post~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obs_IMU_XYZ)))
  "Returns full string definition for message of type 'obs_IMU_XYZ"
  (cl:format cl:nil "int32 timestamp~%uint32 seq~%~%float32 nav_vx~%float32 nav_vy~%~%float32 global_vx~%float32 global_vy~%~%float32 x_pre~%float32 y_pre~%float32 z_pre~%~%float32 dx_pre~%float32 dy_pre~%float32 dz_pre~%~%float32 x_post~%float32 y_post~%float32 z_post~%~%float32 dx_post~%float32 dy_post~%float32 dz_post~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obs_IMU_XYZ>))
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
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obs_IMU_XYZ>))
  "Converts a ROS message object to a list"
  (cl:list 'obs_IMU_XYZ
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':seq (seq msg))
    (cl:cons ':nav_vx (nav_vx msg))
    (cl:cons ':nav_vy (nav_vy msg))
    (cl:cons ':global_vx (global_vx msg))
    (cl:cons ':global_vy (global_vy msg))
    (cl:cons ':x_pre (x_pre msg))
    (cl:cons ':y_pre (y_pre msg))
    (cl:cons ':z_pre (z_pre msg))
    (cl:cons ':dx_pre (dx_pre msg))
    (cl:cons ':dy_pre (dy_pre msg))
    (cl:cons ':dz_pre (dz_pre msg))
    (cl:cons ':x_post (x_post msg))
    (cl:cons ':y_post (y_post msg))
    (cl:cons ':z_post (z_post msg))
    (cl:cons ':dx_post (dx_post msg))
    (cl:cons ':dy_post (dy_post msg))
    (cl:cons ':dz_post (dz_post msg))
))
