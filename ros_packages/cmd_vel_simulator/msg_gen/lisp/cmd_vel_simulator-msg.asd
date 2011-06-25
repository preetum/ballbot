
(cl:in-package :asdf)

(defsystem "cmd_vel_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "drive_cmd" :depends-on ("_package_drive_cmd"))
    (:file "_package_drive_cmd" :depends-on ("_package"))
  ))