
(cl:in-package :asdf)

(defsystem "robot_move_server-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotMoveServerOrder" :depends-on ("_package_RobotMoveServerOrder"))
    (:file "_package_RobotMoveServerOrder" :depends-on ("_package"))
  ))