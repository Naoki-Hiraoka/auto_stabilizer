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
   (foot-step-list step-height-list step-time-list)
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
         (mapcar #'(lambda (sh st)
                     (instance auto_stabilizer::openhrp_autostabilizerservice_stepparam :init :step_height (* sh 1e-3) :step_time st))
                 step-height-list step-time-list)
         ))
  (:set-foot-steps-with-param
   (foot-step-list step-height-list step-time-list)
   "Set foot step with step parameter and wait for step finish.
    For arguments, please see :set-foot-steps-with-param-no-wait documentation."
   (when (send (send self :set-foot-steps-with-param-no-wait foot-step-list step-height-list step-time-list) :operation_return)
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
    &key eefm-rot-damping-gain eefm-pos-damping-gain eefm-pos-time-const eefm-rot-time-const
    &allow-other-keys)
   "Set AutoStabilizer param.
    For arguments, please see (send *ri* :get-auto-stabilizer-param-arguments)."
   (let ((prm (send self :get-auto-stabilizer-param)))
     (send* self :raw-set-auto-stabilizer-param
            (append
             (if (and (memq :eefm-pos-damping-gain args) eefm-pos-damping-gain)
                 (let ((tmp (send prm :eefm_pos_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-damping-gain))
                   (list :eefm-pos-damping-gain tmp)))
             (if (and (memq :eefm-rot-damping-gain args) eefm-rot-damping-gain)
                 (let ((tmp (send prm :eefm_rot_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-damping-gain))
                   (list :eefm-rot-damping-gain tmp)))
             (if (and (memq :eefm-pos-time-const args) eefm-pos-time-const)
                 (let ((tmp (send prm :eefm_pos_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-time-const))
                   (list :eefm-pos-time-const tmp)))
             (if (and (memq :eefm-rot-time-const args) eefm-rot-time-const)
                 (let ((tmp (send prm :eefm_rot_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-time-const))
                   (list :eefm-rot-time-const tmp)))
             args))))
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
  )


