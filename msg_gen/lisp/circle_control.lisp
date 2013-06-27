; Auto-generated. Do not edit!


(cl:in-package AutoNav-msg)


;//! \htmlinclude circle_control.msg.html

(cl:defclass <circle_control> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (goalX
    :reader goalX
    :initarg :goalX
    :type cl:float
    :initform 0.0)
   (goalY
    :reader goalY
    :initarg :goalY
    :type cl:float
    :initform 0.0)
   (errorX
    :reader errorX
    :initarg :errorX
    :type cl:float
    :initform 0.0)
   (errorY
    :reader errorY
    :initarg :errorY
    :type cl:float
    :initform 0.0)
   (PerrorX
    :reader PerrorX
    :initarg :PerrorX
    :type cl:float
    :initform 0.0)
   (PerrorY
    :reader PerrorY
    :initarg :PerrorY
    :type cl:float
    :initform 0.0)
   (PvelX
    :reader PvelX
    :initarg :PvelX
    :type cl:float
    :initform 0.0)
   (PvelY
    :reader PvelY
    :initarg :PvelY
    :type cl:float
    :initform 0.0)
   (VerrX
    :reader VerrX
    :initarg :VerrX
    :type cl:float
    :initform 0.0)
   (VerrY
    :reader VerrY
    :initarg :VerrY
    :type cl:float
    :initform 0.0)
   (VerrA
    :reader VerrA
    :initarg :VerrA
    :type cl:float
    :initform 0.0)
   (CTgainP
    :reader CTgainP
    :initarg :CTgainP
    :type cl:float
    :initform 0.0)
   (CTgainD
    :reader CTgainD
    :initarg :CTgainD
    :type cl:float
    :initform 0.0)
   (ATgainP
    :reader ATgainP
    :initarg :ATgainP
    :type cl:float
    :initform 0.0)
   (ATgainI
    :reader ATgainI
    :initarg :ATgainI
    :type cl:float
    :initform 0.0)
   (ANGgainP
    :reader ANGgainP
    :initarg :ANGgainP
    :type cl:float
    :initform 0.0)
   (ANGgainD
    :reader ANGgainD
    :initarg :ANGgainD
    :type cl:float
    :initform 0.0))
)

(cl:defclass circle_control (<circle_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <circle_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'circle_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AutoNav-msg:<circle_control> is deprecated: use AutoNav-msg:circle_control instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:yaw-val is deprecated.  Use AutoNav-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'goalX-val :lambda-list '(m))
(cl:defmethod goalX-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:goalX-val is deprecated.  Use AutoNav-msg:goalX instead.")
  (goalX m))

(cl:ensure-generic-function 'goalY-val :lambda-list '(m))
(cl:defmethod goalY-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:goalY-val is deprecated.  Use AutoNav-msg:goalY instead.")
  (goalY m))

(cl:ensure-generic-function 'errorX-val :lambda-list '(m))
(cl:defmethod errorX-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:errorX-val is deprecated.  Use AutoNav-msg:errorX instead.")
  (errorX m))

(cl:ensure-generic-function 'errorY-val :lambda-list '(m))
(cl:defmethod errorY-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:errorY-val is deprecated.  Use AutoNav-msg:errorY instead.")
  (errorY m))

(cl:ensure-generic-function 'PerrorX-val :lambda-list '(m))
(cl:defmethod PerrorX-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:PerrorX-val is deprecated.  Use AutoNav-msg:PerrorX instead.")
  (PerrorX m))

(cl:ensure-generic-function 'PerrorY-val :lambda-list '(m))
(cl:defmethod PerrorY-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:PerrorY-val is deprecated.  Use AutoNav-msg:PerrorY instead.")
  (PerrorY m))

(cl:ensure-generic-function 'PvelX-val :lambda-list '(m))
(cl:defmethod PvelX-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:PvelX-val is deprecated.  Use AutoNav-msg:PvelX instead.")
  (PvelX m))

(cl:ensure-generic-function 'PvelY-val :lambda-list '(m))
(cl:defmethod PvelY-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:PvelY-val is deprecated.  Use AutoNav-msg:PvelY instead.")
  (PvelY m))

(cl:ensure-generic-function 'VerrX-val :lambda-list '(m))
(cl:defmethod VerrX-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:VerrX-val is deprecated.  Use AutoNav-msg:VerrX instead.")
  (VerrX m))

(cl:ensure-generic-function 'VerrY-val :lambda-list '(m))
(cl:defmethod VerrY-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:VerrY-val is deprecated.  Use AutoNav-msg:VerrY instead.")
  (VerrY m))

(cl:ensure-generic-function 'VerrA-val :lambda-list '(m))
(cl:defmethod VerrA-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:VerrA-val is deprecated.  Use AutoNav-msg:VerrA instead.")
  (VerrA m))

(cl:ensure-generic-function 'CTgainP-val :lambda-list '(m))
(cl:defmethod CTgainP-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:CTgainP-val is deprecated.  Use AutoNav-msg:CTgainP instead.")
  (CTgainP m))

(cl:ensure-generic-function 'CTgainD-val :lambda-list '(m))
(cl:defmethod CTgainD-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:CTgainD-val is deprecated.  Use AutoNav-msg:CTgainD instead.")
  (CTgainD m))

(cl:ensure-generic-function 'ATgainP-val :lambda-list '(m))
(cl:defmethod ATgainP-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:ATgainP-val is deprecated.  Use AutoNav-msg:ATgainP instead.")
  (ATgainP m))

(cl:ensure-generic-function 'ATgainI-val :lambda-list '(m))
(cl:defmethod ATgainI-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:ATgainI-val is deprecated.  Use AutoNav-msg:ATgainI instead.")
  (ATgainI m))

(cl:ensure-generic-function 'ANGgainP-val :lambda-list '(m))
(cl:defmethod ANGgainP-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:ANGgainP-val is deprecated.  Use AutoNav-msg:ANGgainP instead.")
  (ANGgainP m))

(cl:ensure-generic-function 'ANGgainD-val :lambda-list '(m))
(cl:defmethod ANGgainD-val ((m <circle_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AutoNav-msg:ANGgainD-val is deprecated.  Use AutoNav-msg:ANGgainD instead.")
  (ANGgainD m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <circle_control>) ostream)
  "Serializes a message object of type '<circle_control>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goalX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goalY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'errorX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'errorY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PerrorX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PerrorY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PvelX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PvelY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VerrX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VerrY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VerrA))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CTgainP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CTgainD))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ATgainP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ATgainI))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ANGgainP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ANGgainD))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <circle_control>) istream)
  "Deserializes a message object of type '<circle_control>"
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
    (cl:setf (cl:slot-value msg 'goalX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goalY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'errorX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'errorY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PerrorX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PerrorY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PvelX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PvelY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VerrX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VerrY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VerrA) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CTgainP) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CTgainD) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ATgainP) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ATgainI) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ANGgainP) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ANGgainD) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<circle_control>)))
  "Returns string type for a message object of type '<circle_control>"
  "AutoNav/circle_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'circle_control)))
  "Returns string type for a message object of type 'circle_control"
  "AutoNav/circle_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<circle_control>)))
  "Returns md5sum for a message object of type '<circle_control>"
  "53b73df79b80e8a9ac1f572270c19462")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'circle_control)))
  "Returns md5sum for a message object of type 'circle_control"
  "53b73df79b80e8a9ac1f572270c19462")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<circle_control>)))
  "Returns full string definition for message of type '<circle_control>"
  (cl:format cl:nil "float32 yaw~%~%float32 goalX~%float32 goalY~%~%float32 errorX~%float32 errorY~%~%float32 PerrorX~%float32 PerrorY~%~%float32 PvelX~%float32 PvelY~%~%float32 VerrX~%float32 VerrY~%float32 VerrA~%~%float32 CTgainP~%float32 CTgainD~%~%float32 ATgainP~%float32 ATgainI~%~%float32 ANGgainP~%float32 ANGgainD~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'circle_control)))
  "Returns full string definition for message of type 'circle_control"
  (cl:format cl:nil "float32 yaw~%~%float32 goalX~%float32 goalY~%~%float32 errorX~%float32 errorY~%~%float32 PerrorX~%float32 PerrorY~%~%float32 PvelX~%float32 PvelY~%~%float32 VerrX~%float32 VerrY~%float32 VerrA~%~%float32 CTgainP~%float32 CTgainD~%~%float32 ATgainP~%float32 ATgainI~%~%float32 ANGgainP~%float32 ANGgainD~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <circle_control>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <circle_control>))
  "Converts a ROS message object to a list"
  (cl:list 'circle_control
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':goalX (goalX msg))
    (cl:cons ':goalY (goalY msg))
    (cl:cons ':errorX (errorX msg))
    (cl:cons ':errorY (errorY msg))
    (cl:cons ':PerrorX (PerrorX msg))
    (cl:cons ':PerrorY (PerrorY msg))
    (cl:cons ':PvelX (PvelX msg))
    (cl:cons ':PvelY (PvelY msg))
    (cl:cons ':VerrX (VerrX msg))
    (cl:cons ':VerrY (VerrY msg))
    (cl:cons ':VerrA (VerrA msg))
    (cl:cons ':CTgainP (CTgainP msg))
    (cl:cons ':CTgainD (CTgainD msg))
    (cl:cons ':ATgainP (ATgainP msg))
    (cl:cons ':ATgainI (ATgainI msg))
    (cl:cons ':ANGgainP (ANGgainP msg))
    (cl:cons ':ANGgainD (ANGgainD msg))
))
