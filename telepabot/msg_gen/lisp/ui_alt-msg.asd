
(cl:in-package :asdf)

(defsystem "ui_alt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "src_tf_uialt" :depends-on ("_package_src_tf_uialt"))
    (:file "_package_src_tf_uialt" :depends-on ("_package"))
    (:file "tf_uialt" :depends-on ("_package_tf_uialt"))
    (:file "_package_tf_uialt" :depends-on ("_package"))
  ))