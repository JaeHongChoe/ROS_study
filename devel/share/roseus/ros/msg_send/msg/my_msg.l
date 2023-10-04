;; Auto-generated. Do not edit!


(when (boundp 'msg_send::my_msg)
  (if (not (find-package "MSG_SEND"))
    (make-package "MSG_SEND"))
  (shadow 'my_msg (find-package "MSG_SEND")))
(unless (find-package "MSG_SEND::MY_MSG")
  (make-package "MSG_SEND::MY_MSG"))

(in-package "ROS")
;;//! \htmlinclude my_msg.msg.html


(defclass msg_send::my_msg
  :super ros::object
  :slots (_first_name _last_name _age _score _phone_number _id_number ))

(defmethod msg_send::my_msg
  (:init
   (&key
    ((:first_name __first_name) "")
    ((:last_name __last_name) "")
    ((:age __age) 0)
    ((:score __score) 0)
    ((:phone_number __phone_number) "")
    ((:id_number __id_number) 0)
    )
   (send-super :init)
   (setq _first_name (string __first_name))
   (setq _last_name (string __last_name))
   (setq _age (round __age))
   (setq _score (round __score))
   (setq _phone_number (string __phone_number))
   (setq _id_number (round __id_number))
   self)
  (:first_name
   (&optional __first_name)
   (if __first_name (setq _first_name __first_name)) _first_name)
  (:last_name
   (&optional __last_name)
   (if __last_name (setq _last_name __last_name)) _last_name)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:phone_number
   (&optional __phone_number)
   (if __phone_number (setq _phone_number __phone_number)) _phone_number)
  (:id_number
   (&optional __id_number)
   (if __id_number (setq _id_number __id_number)) _id_number)
  (:serialization-length
   ()
   (+
    ;; string _first_name
    4 (length _first_name)
    ;; string _last_name
    4 (length _last_name)
    ;; int32 _age
    4
    ;; int32 _score
    4
    ;; string _phone_number
    4 (length _phone_number)
    ;; int32 _id_number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _first_name
       (write-long (length _first_name) s) (princ _first_name s)
     ;; string _last_name
       (write-long (length _last_name) s) (princ _last_name s)
     ;; int32 _age
       (write-long _age s)
     ;; int32 _score
       (write-long _score s)
     ;; string _phone_number
       (write-long (length _phone_number) s) (princ _phone_number s)
     ;; int32 _id_number
       (write-long _id_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _first_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _first_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _last_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _last_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _age
     (setq _age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _score
     (setq _score (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _phone_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _phone_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _id_number
     (setq _id_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msg_send::my_msg :md5sum-) "2e13415c121906a5a55d36b792282135")
(setf (get msg_send::my_msg :datatype-) "msg_send/my_msg")
(setf (get msg_send::my_msg :definition-)
      "string first_name
string last_name
int32 age
int32 score
string phone_number
int32 id_number

")



(provide :msg_send/my_msg "2e13415c121906a5a55d36b792282135")


