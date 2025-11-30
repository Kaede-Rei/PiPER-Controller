;; Auto-generated. Do not edit!


(when (boundp 'piper_msgs_srvs::piper_cmd)
  (if (not (find-package "PIPER_MSGS_SRVS"))
    (make-package "PIPER_MSGS_SRVS"))
  (shadow 'piper_cmd (find-package "PIPER_MSGS_SRVS")))
(unless (find-package "PIPER_MSGS_SRVS::PIPER_CMD")
  (make-package "PIPER_MSGS_SRVS::PIPER_CMD"))
(unless (find-package "PIPER_MSGS_SRVS::PIPER_CMDREQUEST")
  (make-package "PIPER_MSGS_SRVS::PIPER_CMDREQUEST"))
(unless (find-package "PIPER_MSGS_SRVS::PIPER_CMDRESPONSE")
  (make-package "PIPER_MSGS_SRVS::PIPER_CMDRESPONSE"))

(in-package "ROS")





(defclass piper_msgs_srvs::piper_cmdRequest
  :super ros::object
  :slots (_command _param1 _param2 _param3 _x _y _z _roll _pitch _yaw ))

(defmethod piper_msgs_srvs::piper_cmdRequest
  (:init
   (&key
    ((:command __command) "")
    ((:param1 __param1) "")
    ((:param2 __param2) "")
    ((:param3 __param3) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _param1 (string __param1))
   (setq _param2 (string __param2))
   (setq _param3 (string __param3))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:param1
   (&optional __param1)
   (if __param1 (setq _param1 __param1)) _param1)
  (:param2
   (&optional __param2)
   (if __param2 (setq _param2 __param2)) _param2)
  (:param3
   (&optional __param3)
   (if __param3 (setq _param3 __param3)) _param3)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _param1
    4 (length _param1)
    ;; string _param2
    4 (length _param2)
    ;; string _param3
    4 (length _param3)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _param1
       (write-long (length _param1) s) (princ _param1 s)
     ;; string _param2
       (write-long (length _param2) s) (princ _param2 s)
     ;; string _param3
       (write-long (length _param3) s) (princ _param3 s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass piper_msgs_srvs::piper_cmdResponse
  :super ros::object
  :slots (_success _message ))

(defmethod piper_msgs_srvs::piper_cmdResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass piper_msgs_srvs::piper_cmd
  :super ros::object
  :slots ())

(setf (get piper_msgs_srvs::piper_cmd :md5sum-) "f532f95cf40949996985d51d8f969194")
(setf (get piper_msgs_srvs::piper_cmd :datatype-) "piper_msgs_srvs/piper_cmd")
(setf (get piper_msgs_srvs::piper_cmd :request) piper_msgs_srvs::piper_cmdRequest)
(setf (get piper_msgs_srvs::piper_cmd :response) piper_msgs_srvs::piper_cmdResponse)

(defmethod piper_msgs_srvs::piper_cmdRequest
  (:response () (instance piper_msgs_srvs::piper_cmdResponse :init)))

(setf (get piper_msgs_srvs::piper_cmdRequest :md5sum-) "f532f95cf40949996985d51d8f969194")
(setf (get piper_msgs_srvs::piper_cmdRequest :datatype-) "piper_msgs_srvs/piper_cmdRequest")
(setf (get piper_msgs_srvs::piper_cmdRequest :definition-)
      "# 请求部分
string command      # 命令
string param1       # 参数1: 对于line/arc是起点坐标字符串
string param2       # 参数2: 对于line/arc是终点坐标字符串
string param3       # 参数3: 备用参数
float64 x           # 目标位置x坐标
float64 y           # 目标位置y坐标
float64 z           # 目标位置z坐标
float64 roll        # 目标姿态roll角
float64 pitch       # 目标姿态pitch角
float64 yaw         # 目标姿态yaw角
---
# 响应部分
bool success        # 执行是否成功
string message      # 返回消息
")

(setf (get piper_msgs_srvs::piper_cmdResponse :md5sum-) "f532f95cf40949996985d51d8f969194")
(setf (get piper_msgs_srvs::piper_cmdResponse :datatype-) "piper_msgs_srvs/piper_cmdResponse")
(setf (get piper_msgs_srvs::piper_cmdResponse :definition-)
      "# 请求部分
string command      # 命令
string param1       # 参数1: 对于line/arc是起点坐标字符串
string param2       # 参数2: 对于line/arc是终点坐标字符串
string param3       # 参数3: 备用参数
float64 x           # 目标位置x坐标
float64 y           # 目标位置y坐标
float64 z           # 目标位置z坐标
float64 roll        # 目标姿态roll角
float64 pitch       # 目标姿态pitch角
float64 yaw         # 目标姿态yaw角
---
# 响应部分
bool success        # 执行是否成功
string message      # 返回消息
")



(provide :piper_msgs_srvs/piper_cmd "f532f95cf40949996985d51d8f969194")


