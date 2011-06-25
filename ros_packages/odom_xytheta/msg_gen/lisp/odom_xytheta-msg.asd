
(cl:in-package :asdf)

(defsystem "odom_xytheta-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "odom_data" :depends-on ("_package_odom_data"))
    (:file "_package_odom_data" :depends-on ("_package"))
  ))