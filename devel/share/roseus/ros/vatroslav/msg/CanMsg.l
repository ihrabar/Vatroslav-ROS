;; Auto-generated. Do not edit!


(when (boundp 'vatroslav::CanMsg)
  (if (not (find-package "VATROSLAV"))
    (make-package "VATROSLAV"))
  (shadow 'CanMsg (find-package "VATROSLAV")))
(unless (find-package "VATROSLAV::CANMSG")
  (make-package "VATROSLAV::CANMSG"))

(in-package "ROS")
;;//! \htmlinclude CanMsg.msg.html


(defclass vatroslav::CanMsg
  :super ros::object
  :slots (_id _data _size _time ))

(defmethod vatroslav::CanMsg
  (:init
   (&key
    ((:id __id) 0)
    ((:data __data) "")
    ((:size __size) 0)
    ((:time __time) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _data (string __data))
   (setq _size (round __size))
   (setq _time (string __time))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; string _data
    4 (length _data)
    ;; uint8 _size
    1
    ;; string _time
    4 (length _time)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;; uint8 _size
       (write-byte _size s)
     ;; string _time
       (write-long (length _time) s) (princ _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _size
     (setq _size (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _time
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _time (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get vatroslav::CanMsg :md5sum-) "4a2f3d2bf382175178ead4deb3882e9c")
(setf (get vatroslav::CanMsg :datatype-) "vatroslav/CanMsg")
(setf (get vatroslav::CanMsg :definition-)
      "uint8 id 
string data  
uint8 size
string time

")



(provide :vatroslav/CanMsg "4a2f3d2bf382175178ead4deb3882e9c")


