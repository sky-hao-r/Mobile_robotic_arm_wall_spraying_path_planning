;; Auto-generated. Do not edit!


(when (boundp 'robot_move_server::RobotMoveServerOrder)
  (if (not (find-package "ROBOT_MOVE_SERVER"))
    (make-package "ROBOT_MOVE_SERVER"))
  (shadow 'RobotMoveServerOrder (find-package "ROBOT_MOVE_SERVER")))
(unless (find-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDER")
  (make-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDER"))
(unless (find-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDERREQUEST")
  (make-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDERREQUEST"))
(unless (find-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDERRESPONSE")
  (make-package "ROBOT_MOVE_SERVER::ROBOTMOVESERVERORDERRESPONSE"))

(in-package "ROS")





(defclass robot_move_server::RobotMoveServerOrderRequest
  :super ros::object
  :slots (_mode _value ))

(defmethod robot_move_server::RobotMoveServerOrderRequest
  (:init
   (&key
    ((:mode __mode) "")
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _value (float __value))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass robot_move_server::RobotMoveServerOrderResponse
  :super ros::object
  :slots (_result ))

(defmethod robot_move_server::RobotMoveServerOrderResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_move_server::RobotMoveServerOrder
  :super ros::object
  :slots ())

(setf (get robot_move_server::RobotMoveServerOrder :md5sum-) "8836a8af838d74e0e72cd61a7e1e78f6")
(setf (get robot_move_server::RobotMoveServerOrder :datatype-) "robot_move_server/RobotMoveServerOrder")
(setf (get robot_move_server::RobotMoveServerOrder :request) robot_move_server::RobotMoveServerOrderRequest)
(setf (get robot_move_server::RobotMoveServerOrder :response) robot_move_server::RobotMoveServerOrderResponse)

(defmethod robot_move_server::RobotMoveServerOrderRequest
  (:response () (instance robot_move_server::RobotMoveServerOrderResponse :init)))

(setf (get robot_move_server::RobotMoveServerOrderRequest :md5sum-) "8836a8af838d74e0e72cd61a7e1e78f6")
(setf (get robot_move_server::RobotMoveServerOrderRequest :datatype-) "robot_move_server/RobotMoveServerOrderRequest")
(setf (get robot_move_server::RobotMoveServerOrderRequest :definition-)
      "string mode
float32 value
---
string result
")

(setf (get robot_move_server::RobotMoveServerOrderResponse :md5sum-) "8836a8af838d74e0e72cd61a7e1e78f6")
(setf (get robot_move_server::RobotMoveServerOrderResponse :datatype-) "robot_move_server/RobotMoveServerOrderResponse")
(setf (get robot_move_server::RobotMoveServerOrderResponse :definition-)
      "string mode
float32 value
---
string result
")



(provide :robot_move_server/RobotMoveServerOrder "8836a8af838d74e0e72cd61a7e1e78f6")


