
(cl:in-package :asdf)

(defsystem "amee-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Velocity" :depends-on ("_package_Velocity"))
    (:file "_package_Velocity" :depends-on ("_package"))
  ))