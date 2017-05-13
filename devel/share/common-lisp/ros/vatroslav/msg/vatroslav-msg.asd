
(cl:in-package :asdf)

(defsystem "vatroslav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CanMsg" :depends-on ("_package_CanMsg"))
    (:file "_package_CanMsg" :depends-on ("_package"))
  ))