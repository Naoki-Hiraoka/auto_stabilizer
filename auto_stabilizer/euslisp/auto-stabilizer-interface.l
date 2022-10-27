(require"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "auto_stabilizer")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "auto_stabilizer")

(def-set-get-param-method
  'auto_stabilizer::Openhrp_AutoStabilizerService_AutoStabilizerParam
  :raw-set-auto-stabilizer-param :get-auto-stabilizer-param :get-auto-stabilizer-param-arguments
  :autostabilizerservice_setautostabilizerparam :autostabilizerservice_getautostabilizerparam)

(defmethod rtm-ros-robot-interface
  (:start-auto-balancer
   ()
   (send self :autostabilizerservice_startAutoBalancer))
  (:stop-auto-balancer
   ()
   (send self :autostabilizerservice_stopAutoBalancer))
  (:go-pos-no-wait
   (xx yy th)
   "Call goPos without wait."
   (send self :autostabilizerservice_goPos :x xx :y yy :th th))
  (:go-pos
   (xx yy th)
   "Call goPos with wait."
   (when (send (send self :go-pos-no-wait xx yy th) :operation_return)
     (send self :wait-foot-steps))) ;; goVelocity中に呼ぶと無限にwaitするため
  (:jump-to-no-wait
   (xx yy zz ts tf)
   "Call jumpTo without wait."
   (send self :autostabilizerservice_jumpTo :x xx :y yy :z zz :ts ts :tf tf))
  (:set-foot-steps-no-wait
   (foot-step-list)
   "Set foot step by default parameters and do not wait for step finish.
    foot-step-list is list of footstep."
   (send self :autostabilizerservice_setfootsteps
         :fs
         (mapcar #'(lambda (f)
                     (send self :eus-footstep->abc-footstep f))
                 foot-step-list)))
  (:set-foot-steps
   (foot-step-list)
   "Set foot step by default parameters and wait for step finish.
    foot-step-list is list of footstep."
   (when (send (send self :set-foot-steps-no-wait foot-step-list) :operation_return)
     (send self :wait-foot-steps))) ;; goVelocity中に呼ぶと無限にwaitするため
  (:set-foot-steps-with-param-no-wait
   (foot-step-list step-height-list step-time-list &optional (swing-end-list (make-list (length foot-step-list) :initial-element nil)))
   "Set foot step with step parameter and do not wait for step finish.
    foot-step-list is list of footstep.
    step-height-list is list of step height[mm].
    step-time-list is list of step time[s]."
   (send self :autostabilizerservice_setfootstepswithparam
         :fs
         (mapcar #'(lambda (f)
                     (send self :eus-footstep->abc-footstep f))
                 foot-step-list)
         :sps
         (mapcar #'(lambda (sh st se)
                     (instance auto_stabilizer::openhrp_autostabilizerservice_stepparam :init :step_height (* sh 1e-3) :step_time st :swing_end se))
                 step-height-list step-time-list swing-end-list)
         ))
  (:set-foot-steps-with-param
   (foot-step-list step-height-list step-time-list &optional (swing-end-list (make-list (length foot-step-list) :initial-element nil)))
   "Set foot step with step parameter and wait for step finish.
    For arguments, please see :set-foot-steps-with-param-no-wait documentation."
   (when (send (send self :set-foot-steps-with-param-no-wait foot-step-list step-height-list step-time-list swing-end-list) :operation_return)
     (send self :wait-foot-steps))) ;; goVelocity中に呼ぶと無限にwaitするため
  (:go-velocity
   (vx vy vth)
   "Call goVelocity. vx[m/s], vy[m/s], and vth[deg/s]"
   (send self :autostabilizerservice_goVelocity :vx vx :vy vy :vth vth))
  (:go-stop
   ()
   "Stop stepping."
   (when (send (send self :autostabilizerservice_goStop) :operation_return)
     (send self :wait-foot-steps)))
  (:wait-foot-steps
   ()
   "Wait for whole footsteps are executed."
   (send self :autostabilizerservice_waitFootSteps))
  (:set-auto-stabilizer-param
   (&rest args
    &key default-zmp-offsets impedance-m-p impedance-d-p impedance-k-p impedance-m-r impedance-d-r impedance-k-r impedance-force-gain impedance-moment-gain support-pgain support-dgain landing-pgain landing-dgain swing-pgain swing-dgain
    &allow-other-keys)
   "Set AutoStabilizer param.
    For arguments, please see (send *ri* :get-auto-stabilizer-param-arguments)."
   (let ((prm (send self :get-auto-stabilizer-param)))
     (send* self :raw-set-auto-stabilizer-param
            (append
             (if (and (memq :default-zmp-offsets args) default-zmp-offsets)
                 (let ((tmp (send prm :default_zmp_offsets)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector default-zmp-offsets))
                   (list :default-zmp-offsets tmp)))
             (if (and (memq :impedance-m-p args) impedance-m-p)
                 (let ((tmp (send prm :impedance_m_p)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-m-p))
                   (list :impedance-m-p tmp)))
             (if (and (memq :impedance-d-p args) impedance-d-p)
                 (let ((tmp (send prm :impedance_d_p)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-d-p))
                   (list :impedance-d-p tmp)))
             (if (and (memq :impedance-k-p args) impedance-k-p)
                 (let ((tmp (send prm :impedance_k_p)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-k-p))
                   (list :impedance-k-p tmp)))
             (if (and (memq :impedance-m-r args) impedance-m-r)
                 (let ((tmp (send prm :impedance_m_r)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-m-r))
                   (list :impedance-m-r tmp)))
             (if (and (memq :impedance-d-r args) impedance-d-r)
                 (let ((tmp (send prm :impedance_d_r)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-d-r))
                   (list :impedance-d-r tmp)))
             (if (and (memq :impedance-k-r args) impedance-k-r)
                 (let ((tmp (send prm :impedance_k_r)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-k-r))
                   (list :impedance-k-r tmp)))
             (if (and (memq :impedance-force-gain args) impedance-force-gain)
                 (let ((tmp (send prm :impedance_force_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-force-gain))
                   (list :impedance-force-gain tmp)))
             (if (and (memq :impedance-moment-gain args) impedance-moment-gain)
                 (let ((tmp (send prm :impedance_moment_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector impedance-moment-gain))
                   (list :impedance-moment-gain tmp)))
             (if (and (memq :support-pgain args) support-pgain)
                 (let ((tmp (send prm :support_pgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector support-pgain))
                   (list :support-pgain tmp)))
             (if (and (memq :support-dgain args) support-dgain)
                 (let ((tmp (send prm :support_dgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector support-dgain))
                   (list :support-dgain tmp)))
             (if (and (memq :landing-pgain args) landing-pgain)
                 (let ((tmp (send prm :landing_pgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector landing-pgain))
                   (list :landing-pgain tmp)))
             (if (and (memq :landing-dgain args) landing-dgain)
                 (let ((tmp (send prm :landing_dgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector landing-dgain))
                   (list :landing-dgain tmp)))
             (if (and (memq :swing-pgain args) swing-pgain)
                 (let ((tmp (send prm :swing_pgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector swing-pgain))
                   (list :swing-pgain tmp)))
             (if (and (memq :swing-dgain args) swing-dgain)
                 (let ((tmp (send prm :swing_dgain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector swing-dgain))
                   (list :swing-dgain tmp)))
             args))))
  (:raw-get-foot-step-states
   ()
   (send (send self :autostabilizerservice_getfootstepstate) :i_param))
  (:get-foot-step-states
   ()
   "Get AutoBalancer foot step state."
   (let ((param (send self :raw-get-foot-step-states)))
     (append
      (mapcan #'(lambda (meth param-name) (list param-name (mapcar #'(lambda (p) (send self :abc-footstep->eus-footstep p)) (send param meth))))
              '(:leg_coords :leg_src_coords :leg_dst_coords)
              '(:leg-coords :leg-src-coords :leg-dst-coords))
      (mapcan #'(lambda (meth param-name) (list param-name (send self :abc-footstep->eus-footstep (send param meth))))
              '(:dst_foot_midcoords)
              '(:dst-foot-midcoords))
      (mapcan #'(lambda (meth param-name)
                  (list param-name (send param meth)))
              '(:support_leg :joint_angle)
              '(:support-leg :joint-angle)))
     ))
  (:get-foot-step-state
   (param-name)
   (cadr (memq param-name (send self :get-foot-step-states)))
   )
  (:eus-footstep->abc-footstep
   (f)
   (instance auto_stabilizer::openhrp_autostabilizerservice_footstep :init
             :pos (scale 1e-3 (send f :worldpos))
             :rot (matrix2quaternion (send f :worldrot))
             :leg (string-downcase (if (find-method f :l/r) (send f :l/r) (send f :name))))
   )
  (:start-st
   ()
   "Start Stabilizer Mode."
   (send self :autostabilizerservice_startstabilizer)
   )
  (:stop-st
   ()
   "Stop Stabilizer Mode."
   (send self :autostabilizerservice_stopstabilizer)
   )
  (:start-impedance
   (limb)
   "Start impedance controller mode.
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :autostabilizerservice_startimpedancecontroller :name (string-downcase name)))
         :start-impedance))
  (:stop-impedance
   (limb)
   "Stop impedance controller mode.
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :autostabilizerservice_stopimpedancecontroller :name (string-downcase name)))
         :stop-impedance))
  (:set-impedance-controller-param
   (limb
    &rest args
    &key m-p d-p k-p m-r d-r k-r force-gain moment-gain)
   "Set impedance controller parameter like (send *ri* :set-impedance-controller-param :rarm :K-p 400).
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs.
    For arguments, please see (send *ri* :get-impedance-controller-param-arguments)."
   (case limb
     (:arms
      (list (send* self :set-impedance-controller-param :rarm args)
            (send* self :set-impedance-controller-param :larm args)))
     (:legs
      (list (send* self :set-impedance-controller-param :rleg args)
            (send* self :set-impedance-controller-param :lleg args)))
     (t
      (let* ((prm (send self :get-auto-stabilizer-param))
             (limb-idx (position (string-downcase limb) (send prm :ee_name) :test #'equal)))
        (send* self :set-auto-stabilizer-param
               (append
                (if (and (memq :m-p args) m-p)
                    (let ((tmp (send prm :impedance_m_p)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) m-p))
                      (list :impedance-m-p (list (tmp . ros::_data)))))
                (if (and (memq :d-p args) d-p)
                    (let ((tmp (send prm :impedance_d_p)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) d-p))
                      (list :impedance-d-p (list (tmp . ros::_data)))))
                (if (and (memq :k-p args) k-p)
                    (let ((tmp (send prm :impedance_k_p)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) k-p))
                      (list :impedance-k-p (list (tmp . ros::_data)))))
                (if (and (memq :m-r args) m-r)
                    (let ((tmp (send prm :impedance_m_r)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) m-r))
                      (list :impedance-m-r (list (tmp . ros::_data)))))
                (if (and (memq :d-r args) d-r)
                    (let ((tmp (send prm :impedance_d_r)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) d-r))
                      (list :impedance-d-r (list (tmp . ros::_data)))))
                (if (and (memq :force-gain args) force-gain)
                    (let ((tmp (send prm :impedance_force_gain)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) (elt force-gain i)))
                      (list :impedance-force-gain (list (tmp . ros::_data)))))
                (if (and (memq :moment-gain args) moment-gain)
                    (let ((tmp (send prm :impedance_moment_gain)))
                      (dotimes (i 3) (setelt (tmp . ros::_data) (+ (* 3 limb-idx) i) (elt moment-gain i)))
                      (list :impedance-moment-gain (list (tmp . ros::_data)))))
                nil))))))
  (:start-whole-body-master-slave
   ()
   (send self :autostabilizerservice_startWholeBodyMasterSlave))
  (:stop-whole-body-master-slave
   ()
   (send self :autostabilizerservice_stopWholeBodyMasterSlave))
  (:calc-dvel-with-velocity-center-offset
   (ang velocity-center-offset)
   "Calculate velocity params for rotating with given velocity center offset.
    Ang : [deg], offset vector [mm]"
   (let* ((default-step-time (send (send self :get-auto-stabilizer-param) :default_step_time))
          (cen (make-cascoords :pos velocity-center-offset))
          (cc (make-cascoords)))
     (send cen :assoc cc)
     (send cen :rotate (deg2rad ang) :z)
     (let ((tf (send (make-coords) :transformation cc)))
       (list
        (/ (* 1e-3 (elt (send tf :worldpos) 0)) default-step-time)
        (/ (* 1e-3 (elt (send tf :worldpos) 1)) default-step-time)
        (/ ang default-step-time)
        ))))
  (:calc-go-velocity-param-from-velocity-center-offset
   (ang velocity-center-offset)
   "Calculate go-velocity velocities from rotation center and rotation angle.
    ang is rotation angle [rad]. velocity-center-offset is velocity center offset [mm] from foot mid coords."
   (let* ((default-step-time (send (send self :get-auto-stabilizer-param) :default_step_time)) ;; [s]
          (cen (make-cascoords :pos velocity-center-offset))
          (cc (make-cascoords)))
     (send cen :assoc cc)
     (send cen :rotate (deg2rad ang) :z)
     (let ((tf (send (make-coords) :transformation cc)))
       (list
        (/ (* 1e-3 (elt (send tf :worldpos) 0)) default-step-time) ;; velx [m/s]
        (/ (* 1e-3 (elt (send tf :worldpos) 1)) default-step-time) ;; vely [m/s]
        (/ ang default-step-time) ;; velth [rad/s]
        ))))
  )


