
(cl:in-package :asdf)

(defsystem "piper_msgs_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "piper_cmd" :depends-on ("_package_piper_cmd"))
    (:file "_package_piper_cmd" :depends-on ("_package"))
  ))