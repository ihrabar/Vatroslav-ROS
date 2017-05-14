; Auto-generated. Do not edit!


(cl:in-package vatroslav-msg)


;//! \htmlinclude CanMsg.msg.html

(cl:defclass <CanMsg> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform "")
   (size
    :reader size
    :initarg :size
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:string
    :initform ""))
)

(cl:defclass CanMsg (<CanMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CanMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CanMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vatroslav-msg:<CanMsg> is deprecated: use vatroslav-msg:CanMsg instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CanMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vatroslav-msg:id-val is deprecated.  Use vatroslav-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <CanMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vatroslav-msg:data-val is deprecated.  Use vatroslav-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <CanMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vatroslav-msg:size-val is deprecated.  Use vatroslav-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <CanMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vatroslav-msg:time-val is deprecated.  Use vatroslav-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CanMsg>) ostream)
  "Serializes a message object of type '<CanMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'time))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CanMsg>) istream)
  "Deserializes a message object of type '<CanMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'time) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CanMsg>)))
  "Returns string type for a message object of type '<CanMsg>"
  "vatroslav/CanMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CanMsg)))
  "Returns string type for a message object of type 'CanMsg"
  "vatroslav/CanMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CanMsg>)))
  "Returns md5sum for a message object of type '<CanMsg>"
  "4a2f3d2bf382175178ead4deb3882e9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CanMsg)))
  "Returns md5sum for a message object of type 'CanMsg"
  "4a2f3d2bf382175178ead4deb3882e9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CanMsg>)))
  "Returns full string definition for message of type '<CanMsg>"
  (cl:format cl:nil "uint8 id ~%string data  ~%uint8 size~%string time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CanMsg)))
  "Returns full string definition for message of type 'CanMsg"
  (cl:format cl:nil "uint8 id ~%string data  ~%uint8 size~%string time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CanMsg>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'data))
     1
     4 (cl:length (cl:slot-value msg 'time))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CanMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CanMsg
    (cl:cons ':id (id msg))
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
    (cl:cons ':time (time msg))
))
