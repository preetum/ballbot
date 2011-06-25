
(cl:in-package :asdf)

(defsystem "cmd_vel_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "goal_msg" :depends-on ("_package_goal_msg"))
    (:file "_package_goal_msg" :depends-on ("_package"))
    (:file "drive_cmd" :depends-on ("_package_drive_cmd"))
    (:file "_package_drive_cmd" :depends-on ("_package"))
  ))