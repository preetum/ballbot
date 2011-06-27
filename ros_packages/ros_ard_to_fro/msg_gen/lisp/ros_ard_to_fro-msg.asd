
(cl:in-package :asdf)

(defsystem "ros_ard_to_fro-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "odom_data" :depends-on ("_package_odom_data"))
    (:file "_package_odom_data" :depends-on ("_package"))
    (:file "drive_cmd" :depends-on ("_package_drive_cmd"))
    (:file "_package_drive_cmd" :depends-on ("_package"))
  ))