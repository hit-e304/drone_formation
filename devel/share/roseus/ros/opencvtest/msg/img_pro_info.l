;; Auto-generated. Do not edit!


(when (boundp 'opencvtest::img_pro_info)
  (if (not (find-package "OPENCVTEST"))
    (make-package "OPENCVTEST"))
  (shadow 'img_pro_info (find-package "OPENCVTEST")))
(unless (find-package "OPENCVTEST::IMG_PRO_INFO")
  (make-package "OPENCVTEST::IMG_PRO_INFO"))

(in-package "ROS")
;;//! \htmlinclude img_pro_info.msg.html


(defclass opencvtest::img_pro_info
  :super ros::object
  :slots (_find_obs_flag _dis _pos_left _pos_right _x_pos _y_pos ))

(defmethod opencvtest::img_pro_info
  (:init
   (&key
    ((:find_obs_flag __find_obs_flag) nil)
    ((:dis __dis) 0.0)
    ((:pos_left __pos_left) 0)
    ((:pos_right __pos_right) 0)
    ((:x_pos __x_pos) 0)
    ((:y_pos __y_pos) 0)
    )
   (send-super :init)
   (setq _find_obs_flag __find_obs_flag)
   (setq _dis (float __dis))
   (setq _pos_left (round __pos_left))
   (setq _pos_right (round __pos_right))
   (setq _x_pos (round __x_pos))
   (setq _y_pos (round __y_pos))
   self)
  (:find_obs_flag
   (&optional __find_obs_flag)
   (if __find_obs_flag (setq _find_obs_flag __find_obs_flag)) _find_obs_flag)
  (:dis
   (&optional __dis)
   (if __dis (setq _dis __dis)) _dis)
  (:pos_left
   (&optional __pos_left)
   (if __pos_left (setq _pos_left __pos_left)) _pos_left)
  (:pos_right
   (&optional __pos_right)
   (if __pos_right (setq _pos_right __pos_right)) _pos_right)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:serialization-length
   ()
   (+
    ;; bool _find_obs_flag
    1
    ;; float64 _dis
    8
    ;; int32 _pos_left
    4
    ;; int32 _pos_right
    4
    ;; int32 _x_pos
    4
    ;; int32 _y_pos
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _find_obs_flag
       (if _find_obs_flag (write-byte -1 s) (write-byte 0 s))
     ;; float64 _dis
       (sys::poke _dis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _pos_left
       (write-long _pos_left s)
     ;; int32 _pos_right
       (write-long _pos_right s)
     ;; int32 _x_pos
       (write-long _x_pos s)
     ;; int32 _y_pos
       (write-long _y_pos s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _find_obs_flag
     (setq _find_obs_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _dis
     (setq _dis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _pos_left
     (setq _pos_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pos_right
     (setq _pos_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get opencvtest::img_pro_info :md5sum-) "254e05080fb643551976301900a3b655")
(setf (get opencvtest::img_pro_info :datatype-) "opencvtest/img_pro_info")
(setf (get opencvtest::img_pro_info :definition-)
      "    bool find_obs_flag
    float64 dis
    int32 pos_left
    int32 pos_right
    int32 x_pos
    int32 y_pos
")



(provide :opencvtest/img_pro_info "254e05080fb643551976301900a3b655")


