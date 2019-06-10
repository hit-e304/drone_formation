;; Auto-generated. Do not edit!


(when (boundp 'opencvtest::contours)
  (if (not (find-package "OPENCVTEST"))
    (make-package "OPENCVTEST"))
  (shadow 'contours (find-package "OPENCVTEST")))
(unless (find-package "OPENCVTEST::CONTOURS")
  (make-package "OPENCVTEST::CONTOURS"))

(in-package "ROS")
;;//! \htmlinclude contours.msg.html


(defclass opencvtest::contours
  :super ros::object
  :slots (_pro_info ))

(defmethod opencvtest::contours
  (:init
   (&key
    ((:pro_info __pro_info) (instance opencvtest::img_pro_info :init))
    )
   (send-super :init)
   (setq _pro_info __pro_info)
   self)
  (:pro_info
   (&rest __pro_info)
   (if (keywordp (car __pro_info))
       (send* _pro_info __pro_info)
     (progn
       (if __pro_info (setq _pro_info (car __pro_info)))
       _pro_info)))
  (:serialization-length
   ()
   (+
    ;; opencvtest/img_pro_info _pro_info
    (send _pro_info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; opencvtest/img_pro_info _pro_info
       (send _pro_info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; opencvtest/img_pro_info _pro_info
     (send _pro_info :deserialize buf ptr-) (incf ptr- (send _pro_info :serialization-length))
   ;;
   self)
  )

(setf (get opencvtest::contours :md5sum-) "80338e53bc763a94f85a1a487d6d2c70")
(setf (get opencvtest::contours :datatype-) "opencvtest/contours")
(setf (get opencvtest::contours :definition-)
      "img_pro_info pro_info
================================================================================
MSG: opencvtest/img_pro_info
    bool find_obs_flag
    float64 dis
    int32 pos_left
    int32 pos_right
    int32 x_pos
    int32 y_pos
")



(provide :opencvtest/contours "80338e53bc763a94f85a1a487d6d2c70")


