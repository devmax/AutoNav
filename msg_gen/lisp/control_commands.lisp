; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude control_commands.msg.html

(cl:defclass <control_commands> (roslisp-msg-protocol:ros-message)
  ((error_x
    :reader error_x
    :initarg :error_x
    :type cl:float
    :initform 0.0)
   (error_y
    :reader error_y
    :initarg :error_y
    :type cl:float
    :initform 0.0)
   (error_z
    :reader error_z
    :initarg :error_z
    :type cl:float
    :initform 0.0)
   (error_yaw
    :reader error_yaw
    :initarg :error_yaw
    :type cl:float
    :initform 0.0)
   (d_error_x
    :reader d_error_x
    :initarg :d_error_x
    :type cl:float
    :initform 0.0)
   (d_error_y
    :reader d_error_y
    :initarg :d_error_y
    :type cl:float
    :initform 0.0)
   (d_error_z
    :reader d_error_z
    :initarg :d_error_z
    :type cl:float
    :initform 0.0)
   (d_error_yaw
    :reader d_error_yaw
    :initarg :d_error_yaw
    :type cl:float
    :initform 0.0)
   (proj_error_x
    :reader proj_error_x
    :initarg :proj_error_x
    :type cl:float
    :initform 0.0)
   (proj_error_y
    :reader proj_error_y
    :initarg :proj_error_y
    :type cl:float
    :initform 0.0)
   (pterm_x
    :reader pterm_x
    :initarg :pterm_x
    :type cl:float
    :initform 0.0)
   (pterm_y
    :reader pterm_y
    :initarg :pterm_y
    :type cl:float
    :initform 0.0)
   (pterm_z
    :reader pterm_z
    :initarg :pterm_z
    :type cl:float
    :initform 0.0)
   (pterm_yaw
    :reader pterm_yaw
    :initarg :pterm_yaw
    :type cl:float
    :initform 0.0)
   (dterm_x
    :reader dterm_x
    :initarg :dterm_x
    :type cl:float
    :initform 0.0)
   (dterm_y
    :reader dterm_y
    :initarg :dterm_y
    :type cl:float
    :initform 0.0)
   (dterm_z
    :reader dterm_z
    :initarg :dterm_z
    :type cl:float
    :initform 0.0)
   (dterm_yaw
    :reader dterm_yaw
    :initarg :dterm_yaw
    :type cl:float
    :initform 0.0)
   (vel_x
    :reader vel_x
    :initarg :vel_x
    :type cl:float
    :initform 0.0)
   (vel_y
    :reader vel_y
    :initarg :vel_y
    :type cl:float
    :initform 0.0)
   (vel_z
    :reader vel_z
    :initarg :vel_z
    :type cl:float
    :initform 0.0)
   (vel_yaw
    :reader vel_yaw
    :initarg :vel_yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass control_commands (<control_commands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_commands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_commands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<control_commands> is deprecated: use AutoNav-msg:control_commands instead.")))

(cl:ensure-generic-function 'error_x-val :lambda-list '(m))
(cl:defmethod error_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:error_x-val is deprecated.  Use AutoNav-msg:error_x instead.")
  (error_x m))

(cl:ensure-generic-function 'error_y-val :lambda-list '(m))
(cl:defmethod error_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:error_y-val is deprecated.  Use AutoNav-msg:error_y instead.")
  (error_y m))

(cl:ensure-generic-function 'error_z-val :lambda-list '(m))
(cl:defmethod error_z-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:error_z-val is deprecated.  Use AutoNav-msg:error_z instead.")
  (error_z m))

(cl:ensure-generic-function 'error_yaw-val :lambda-list '(m))
(cl:defmethod error_yaw-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:error_yaw-val is deprecated.  Use AutoNav-msg:error_yaw instead.")
  (error_yaw m))

(cl:ensure-generic-function 'd_error_x-val :lambda-list '(m))
(cl:defmethod d_error_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:d_error_x-val is deprecated.  Use AutoNav-msg:d_error_x instead.")
  (d_error_x m))

(cl:ensure-generic-function 'd_error_y-val :lambda-list '(m))
(cl:defmethod d_error_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:d_error_y-val is deprecated.  Use AutoNav-msg:d_error_y instead.")
  (d_error_y m))

(cl:ensure-generic-function 'd_error_z-val :lambda-list '(m))
(cl:defmethod d_error_z-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:d_error_z-val is deprecated.  Use AutoNav-msg:d_error_z instead.")
  (d_error_z m))

(cl:ensure-generic-function 'd_error_yaw-val :lambda-list '(m))
(cl:defmethod d_error_yaw-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:d_error_yaw-val is deprecated.  Use AutoNav-msg:d_error_yaw instead.")
  (d_error_yaw m))

(cl:ensure-generic-function 'proj_error_x-val :lambda-list '(m))
(cl:defmethod proj_error_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:proj_error_x-val is deprecated.  Use AutoNav-msg:proj_error_x instead.")
  (proj_error_x m))

(cl:ensure-generic-function 'proj_error_y-val :lambda-list '(m))
(cl:defmethod proj_error_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:proj_error_y-val is deprecated.  Use AutoNav-msg:proj_error_y instead.")
  (proj_error_y m))

(cl:ensure-generic-function 'pterm_x-val :lambda-list '(m))
(cl:defmethod pterm_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pterm_x-val is deprecated.  Use AutoNav-msg:pterm_x instead.")
  (pterm_x m))

(cl:ensure-generic-function 'pterm_y-val :lambda-list '(m))
(cl:defmethod pterm_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pterm_y-val is deprecated.  Use AutoNav-msg:pterm_y instead.")
  (pterm_y m))

(cl:ensure-generic-function 'pterm_z-val :lambda-list '(m))
(cl:defmethod pterm_z-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pterm_z-val is deprecated.  Use AutoNav-msg:pterm_z instead.")
  (pterm_z m))

(cl:ensure-generic-function 'pterm_yaw-val :lambda-list '(m))
(cl:defmethod pterm_yaw-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:pterm_yaw-val is deprecated.  Use AutoNav-msg:pterm_yaw instead.")
  (pterm_yaw m))

(cl:ensure-generic-function 'dterm_x-val :lambda-list '(m))
(cl:defmethod dterm_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dterm_x-val is deprecated.  Use AutoNav-msg:dterm_x instead.")
  (dterm_x m))

(cl:ensure-generic-function 'dterm_y-val :lambda-list '(m))
(cl:defmethod dterm_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dterm_y-val is deprecated.  Use AutoNav-msg:dterm_y instead.")
  (dterm_y m))

(cl:ensure-generic-function 'dterm_z-val :lambda-list '(m))
(cl:defmethod dterm_z-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dterm_z-val is deprecated.  Use AutoNav-msg:dterm_z instead.")
  (dterm_z m))

(cl:ensure-generic-function 'dterm_yaw-val :lambda-list '(m))
(cl:defmethod dterm_yaw-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:dterm_yaw-val is deprecated.  Use AutoNav-msg:dterm_yaw instead.")
  (dterm_yaw m))

(cl:ensure-generic-function 'vel_x-val :lambda-list '(m))
(cl:defmethod vel_x-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vel_x-val is deprecated.  Use AutoNav-msg:vel_x instead.")
  (vel_x m))

(cl:ensure-generic-function 'vel_y-val :lambda-list '(m))
(cl:defmethod vel_y-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vel_y-val is deprecated.  Use AutoNav-msg:vel_y instead.")
  (vel_y m))

(cl:ensure-generic-function 'vel_z-val :lambda-list '(m))
(cl:defmethod vel_z-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vel_z-val is deprecated.  Use AutoNav-msg:vel_z instead.")
  (vel_z m))

(cl:ensure-generic-function 'vel_yaw-val :lambda-list '(m))
(cl:defmethod vel_yaw-val ((m <control_commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:vel_yaw-val is deprecated.  Use AutoNav-msg:vel_yaw instead.")
  (vel_yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_commands>) ostream)
  "Serializes a message object of type '<control_commands>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_error_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_error_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_error_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_error_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proj_error_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proj_error_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pterm_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pterm_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pterm_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pterm_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dterm_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dterm_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dterm_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dterm_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_commands>) istream)
  "Deserializes a message object of type '<control_commands>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_error_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_error_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_error_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_error_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proj_error_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proj_error_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pterm_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pterm_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pterm_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pterm_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dterm_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dterm_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dterm_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dterm_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_commands>)))
  "Returns string type for a message object of type '<control_commands>"
  "AutoNav/control_commands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_commands)))
  "Returns string type for a message object of type 'control_commands"
  "AutoNav/control_commands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_commands>)))
  "Returns md5sum for a message object of type '<control_commands>"
  "89daaa5cf29061dcd0b2da457fee8393")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_commands)))
  "Returns md5sum for a message object of type 'control_commands"
  "89daaa5cf29061dcd0b2da457fee8393")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_commands>)))
  "Returns full string definition for message of type '<control_commands>"
  (cl:format cl:nil "~%float32 error_x~%float32 error_y~%float32 error_z~%float32 error_yaw~%~%float32 d_error_x~%float32 d_error_y~%float32 d_error_z~%float32 d_error_yaw~%~%float32 proj_error_x~%float32 proj_error_y~%~%float32 pterm_x~%float32 pterm_y~%float32 pterm_z~%float32 pterm_yaw~%~%float32 dterm_x~%float32 dterm_y~%float32 dterm_z~%float32 dterm_yaw~%~%float32 vel_x~%float32 vel_y~%float32 vel_z~%float32 vel_yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_commands)))
  "Returns full string definition for message of type 'control_commands"
  (cl:format cl:nil "~%float32 error_x~%float32 error_y~%float32 error_z~%float32 error_yaw~%~%float32 d_error_x~%float32 d_error_y~%float32 d_error_z~%float32 d_error_yaw~%~%float32 proj_error_x~%float32 proj_error_y~%~%float32 pterm_x~%float32 pterm_y~%float32 pterm_z~%float32 pterm_yaw~%~%float32 dterm_x~%float32 dterm_y~%float32 dterm_z~%float32 dterm_yaw~%~%float32 vel_x~%float32 vel_y~%float32 vel_z~%float32 vel_yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_commands>))
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
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_commands>))
  "Converts a ROS message object to a list"
  (cl:list 'control_commands
    (cl:cons ':error_x (error_x msg))
    (cl:cons ':error_y (error_y msg))
    (cl:cons ':error_z (error_z msg))
    (cl:cons ':error_yaw (error_yaw msg))
    (cl:cons ':d_error_x (d_error_x msg))
    (cl:cons ':d_error_y (d_error_y msg))
    (cl:cons ':d_error_z (d_error_z msg))
    (cl:cons ':d_error_yaw (d_error_yaw msg))
    (cl:cons ':proj_error_x (proj_error_x msg))
    (cl:cons ':proj_error_y (proj_error_y msg))
    (cl:cons ':pterm_x (pterm_x msg))
    (cl:cons ':pterm_y (pterm_y msg))
    (cl:cons ':pterm_z (pterm_z msg))
    (cl:cons ':pterm_yaw (pterm_yaw msg))
    (cl:cons ':dterm_x (dterm_x msg))
    (cl:cons ':dterm_y (dterm_y msg))
    (cl:cons ':dterm_z (dterm_z msg))
    (cl:cons ':dterm_yaw (dterm_yaw msg))
    (cl:cons ':vel_x (vel_x msg))
    (cl:cons ':vel_y (vel_y msg))
    (cl:cons ':vel_z (vel_z msg))
    (cl:cons ':vel_yaw (vel_yaw msg))
))
