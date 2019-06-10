
(cl:in-package :asdf)

(defsystem "opencvtest-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "contours" :depends-on ("_package_contours"))
    (:file "_package_contours" :depends-on ("_package"))
    (:file "img_pro_info" :depends-on ("_package_img_pro_info"))
    (:file "_package_img_pro_info" :depends-on ("_package"))
  ))