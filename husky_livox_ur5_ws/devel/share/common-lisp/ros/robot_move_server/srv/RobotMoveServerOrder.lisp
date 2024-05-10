; Auto-generated. Do not edit!


(cl:in-package robot_move_server-srv)


;//! \htmlinclude RobotMoveServerOrder-request.msg.html

(cl:defclass <RobotMoveServerOrder-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass RobotMoveServerOrder-request (<RobotMoveServerOrder-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMoveServerOrder-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMoveServerOrder-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_move_server-srv:<RobotMoveServerOrder-request> is deprecated: use robot_move_server-srv:RobotMoveServerOrder-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <RobotMoveServerOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_move_server-srv:mode-val is deprecated.  Use robot_move_server-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <RobotMoveServerOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_move_server-srv:value-val is deprecated.  Use robot_move_server-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMoveServerOrder-request>) ostream)
  "Serializes a message object of type '<RobotMoveServerOrder-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMoveServerOrder-request>) istream)
  "Deserializes a message object of type '<RobotMoveServerOrder-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMoveServerOrder-request>)))
  "Returns string type for a service object of type '<RobotMoveServerOrder-request>"
  "robot_move_server/RobotMoveServerOrderRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMoveServerOrder-request)))
  "Returns string type for a service object of type 'RobotMoveServerOrder-request"
  "robot_move_server/RobotMoveServerOrderRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMoveServerOrder-request>)))
  "Returns md5sum for a message object of type '<RobotMoveServerOrder-request>"
  "8836a8af838d74e0e72cd61a7e1e78f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMoveServerOrder-request)))
  "Returns md5sum for a message object of type 'RobotMoveServerOrder-request"
  "8836a8af838d74e0e72cd61a7e1e78f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMoveServerOrder-request>)))
  "Returns full string definition for message of type '<RobotMoveServerOrder-request>"
  (cl:format cl:nil "string mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMoveServerOrder-request)))
  "Returns full string definition for message of type 'RobotMoveServerOrder-request"
  (cl:format cl:nil "string mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMoveServerOrder-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMoveServerOrder-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMoveServerOrder-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude RobotMoveServerOrder-response.msg.html

(cl:defclass <RobotMoveServerOrder-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass RobotMoveServerOrder-response (<RobotMoveServerOrder-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMoveServerOrder-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMoveServerOrder-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_move_server-srv:<RobotMoveServerOrder-response> is deprecated: use robot_move_server-srv:RobotMoveServerOrder-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <RobotMoveServerOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_move_server-srv:result-val is deprecated.  Use robot_move_server-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMoveServerOrder-response>) ostream)
  "Serializes a message object of type '<RobotMoveServerOrder-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMoveServerOrder-response>) istream)
  "Deserializes a message object of type '<RobotMoveServerOrder-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMoveServerOrder-response>)))
  "Returns string type for a service object of type '<RobotMoveServerOrder-response>"
  "robot_move_server/RobotMoveServerOrderResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMoveServerOrder-response)))
  "Returns string type for a service object of type 'RobotMoveServerOrder-response"
  "robot_move_server/RobotMoveServerOrderResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMoveServerOrder-response>)))
  "Returns md5sum for a message object of type '<RobotMoveServerOrder-response>"
  "8836a8af838d74e0e72cd61a7e1e78f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMoveServerOrder-response)))
  "Returns md5sum for a message object of type 'RobotMoveServerOrder-response"
  "8836a8af838d74e0e72cd61a7e1e78f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMoveServerOrder-response>)))
  "Returns full string definition for message of type '<RobotMoveServerOrder-response>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMoveServerOrder-response)))
  "Returns full string definition for message of type 'RobotMoveServerOrder-response"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMoveServerOrder-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMoveServerOrder-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMoveServerOrder-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotMoveServerOrder)))
  'RobotMoveServerOrder-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotMoveServerOrder)))
  'RobotMoveServerOrder-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMoveServerOrder)))
  "Returns string type for a service object of type '<RobotMoveServerOrder>"
  "robot_move_server/RobotMoveServerOrder")