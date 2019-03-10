
(cl:in-package :asdf)

(defsystem "parking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :parking-msg
)
  :components ((:file "_package")
    (:file "maneuver" :depends-on ("_package_maneuver"))
    (:file "_package_maneuver" :depends-on ("_package"))
  ))