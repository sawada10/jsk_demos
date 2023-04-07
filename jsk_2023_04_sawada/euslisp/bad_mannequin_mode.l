;; 必要なmsgの型をロードする
(ros::load-ros-manifest "pr2_mechanism_msgs")
(ros::load-ros-manifest "speech_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")
;; ここで*ri* *pr2*が作られる
(pr2-init)

(defun mannequin-mode (flag)
  ;; ノードを宣言する
  (ros::roseus "mannequin_mode")
  ;; サービスが使えるようになるまで待機する
  (ros::wait-for-service "/pr2_controller_manager/switch_controller")
  ;; requestのメッセージを初期化して作る
  (setq req (instance pr2_mechanism_msgs::SwitchControllerRequest :init))
  ;; requestのメッセージに中身を入れる
  ;; flagがtのときにはマネキンモードにする
  (if flag
	  (progn
		(send req :start_controllers (list "r_arm_controller_loose"))
		(send req :stop_controllers (list "r_arm_controller"))
		(send *ri* :speak-en "change to mannequin mode")
		)
	;; flagがnilのときはマネキンモードをやめる
	(progn
	  (send req :start_controllers (list "r_arm_controller"))
	  (send req :stop_controllers (list "r_arm_controller_loose"))
	  (send *ri* :speak-en "stop mannequin mode")
	  )
	)
  
  ;; /pr2_controller_manager/switch_controllerにreqを流す?
  (setq res (ros::service-call "/pr2_controller_manager/switch_controller" req t))
  ;; 話す

  )


(defun voice-cb (msg)
  (when msg
    (setq *msg* msg)
    (setq *voice-data* (send *msg* :transcript))
    (setq *voice-data-flag* t)
    (format t "voice-data: ~A ~%" *voice-data*)
	(format t "condition : ~A ~%" (string-equal "ストップ" (car *voice-data*)))
	(cond
	 ((equal (list "OK") *voice-data*)
	  (progn
		(format t "ok was said")
		(mannequin-mode t)
		)
	  )
	 ((equal (list "ストップ") *voice-data*)
	  ;;(progn
		;;(format t "stop was said")
		;;(mannequin-mode nil)
	  ;;)
	  (format t "stop was said")
	  )
	 (t
	  (print "hoge"))
	 )
	)
  )

(defun dialogue-func ()
  (ros::roseus "speech")
  ;; とりあえずいれておくやつ
  (ros::unsubscribe "/speech_to_text")
  ;; voiceが来たかどうかのflag
  (setq *voice-data-flag* nil)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'voice-cb)
  ;; 音声が来てる間
  (until *voice-data-flag*
		;;cb関数を1回実行する
		(ros::spin-once)
		;; 時間を貯める
		(ros::sleep)
	)
  )

(dialogue-func)
(format t "condition at the end: ~A ~%" (equal (list "ストップ") *voice-data*))
(format t "condition2 at the end: ~A ~%" (equal '("ストップ") *voice-data*))


