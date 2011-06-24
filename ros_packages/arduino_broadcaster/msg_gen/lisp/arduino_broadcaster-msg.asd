
(cl:in-package :asdf)

(defsystem "arduino_broadcaster-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "arduino_data" :depends-on ("_package_arduino_data"))
    (:file "_package_arduino_data" :depends-on ("_package"))
  ))