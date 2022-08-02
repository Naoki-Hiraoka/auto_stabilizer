(require"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "auto_stabilizer")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "auto_stabilizer")

(def-set-get-param-method
  'auto_stabilizer::Openhrp_AutoStabilizerService_GaitGeneratorParam
  :raw-set-gait-generator-param :get-gait-generator-param :get-gait-generator-param-arguments
  :autostabilizerservice_setgaitgeneratorparam :autostabilizerservice_getgaitgeneratorparam)
(def-set-get-param-method
  'auto_stabilizer::Openhrp_AutoStabilizerService_AutoBalancerParam
  :raw-set-auto-balancer-param :get-auto-balancer-param :get-auto-balancer-param-arguments
  :autostabilizerservice_setAutoBalancerparam :autostabilizerservice_getAutoBalancerparam)
(def-set-get-param-method
  'auto_stabilizer::Openhrp_AutoStabilizerService_StabilizerParam
  :raw-set-st-param :get-st-param :get-st-param-arguments
  :autostabilizerservice_setstabilizerparam :autostabilizerservice_getstabilizerparam)

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
  (:jump-to-no-wait
   (xx yy zz ts tf)
   "Call jumpTo without wait."
   (send self :autostabilizerservice_jumpTo :x xx :y yy :z zz :ts ts :tf tf))
  (:set-foot-steps-no-wait
   (foot-step-list &key (overwrite-footstep-index 0))
   "Set foot step by default parameters and do not wait for step finish.
    foot-step-list is list of footstep (only biped) or list of list of footstep.
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking."
   (unless (listp (car foot-step-list))
     (setq foot-step-list (mapcar #'(lambda (fs) (list fs)) foot-step-list)))
   (send self :autostabilizerservice_setfootsteps
         :fss
         (mapcar #'(lambda (fs)
                     (instance auto_stabilizer::openhrp_autostabilizerservice_footsteps :init
                               :fs
                               (mapcar #'(lambda (f)
                                           (send self :eus-footstep->abc-footstep f))
                                       fs)))
                 foot-step-list)
         :overwrite_fs_idx overwrite-footstep-index))
  (:set-foot-steps-with-param-no-wait
   (foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list &key (overwrite-footstep-index 0))
   "Set foot step with step parameter and do not wait for step finish.
    foot-step-list is list of footstep (only biped) or list of list of footstep.
    step-height-list is list of step height (only biped) or list of list of step height.
    step-time-list is list of step time (only biped) or list of list of step time.
    toe-angle-list is list of toe angle (only biped) or list of list of toe angle.
    heel-angle-list is list of heel angle (only biped) or list of list of heel angle."
   (unless (listp (car foot-step-list))
     (setq foot-step-list (mapcar #'(lambda (fs) (list fs)) foot-step-list)
           step-height-list (mapcar #'(lambda (sh) (list sh)) step-height-list)
           step-time-list (mapcar #'(lambda (st) (list st)) step-time-list)
           toe-angle-list (mapcar #'(lambda (ta) (list ta)) toe-angle-list)
           heel-angle-list (mapcar #'(lambda (ha) (list ha)) heel-angle-list)))
   (send self :autostabilizerservice_setfootstepswithparam
         :fss
         (mapcar #'(lambda (fs)
                     (instance auto_stabilizer::openhrp_autostabilizerservice_footsteps :init
                               :fs
                               (mapcar #'(lambda (f)
                                           (send self :eus-footstep->abc-footstep f))
                                       fs)))
                 foot-step-list)
         :spss
         (mapcar #'(lambda (shs sts tas has)
                     (instance auto_stabilizer::openhrp_autostabilizerservice_stepparams :init
                               :sps
                               (mapcar #'(lambda (sh st ta ha)
                                           (instance auto_stabilizer::openhrp_autostabilizerservice_stepparam :init :step_height (* sh 1e-3) :step_time st :toe_angle ta :heel_angle ha))
                                       shs sts tas has)))
                 step-height-list step-time-list toe-angle-list heel-angle-list)
         :overwrite_fs_idx overwrite-footstep-index
         ))
  (:go-velocity
   (vx vy vth)
   "Call goVelocity. vx[m/s], vy[m/s], and vth[deg/s]"
   (send self :autostabilizerservice_goVelocity :vx vx :vy vy :vth vth))
  (:go-stop
   ()
   "Stop stepping."
   (send self :autostabilizerservice_goStop))
  (:wait-foot-steps
   ()
   "Wait for whole footsteps are executed."
   (send self :autostabilizerservice_waitFootSteps))
  (:set-gait-generator-param
   (&rest args
    &key default-orbit-type
         leg-default-translate-pos ;; [mm]
         stride-parameter
    &allow-other-keys)
   "Set gait generator param.
    For arguments, please see (send *ri* :get-gait-generator-param-arguments)."
   (let ((gg-prm (if (or (memq :leg-default-translate-pos args) (memq :stride-parameter args)) (send self :get-gait-generator-param))))
     (send* self :raw-set-gait-generator-param
            (append
             (if (memq :default-orbit-type args)
                 (list :default-orbit-type
                       (if (or (null default-orbit-type) (integerp default-orbit-type))
                           default-orbit-type
                         (let ((ot (read-from-string (format nil "AUTO_STABILIZER::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE::*~A*" (string-downcase default-orbit-type)))))
                           (if (boundp ot)
                               (eval ot)
                             (error ";; no such :default-orbit-type ~A in :set-gait-generator-param~%" default-orbit-type))
                           ))))
             (if (memq :leg-default-translate-pos args)
                 (let ((lo (copy-object (send gg-prm :leg_default_translate_pos))))
                   (setq (lo . ros::_data) (scale 1e-3 (apply #'concatenate float-vector leg-default-translate-pos)))
                   (list :leg-default-translate-pos lo)))
             (if (memq :stride-parameter args)
                 (let ((sprm (send gg-prm :stride_parameter)))
                   (cond
                    ((= (length sprm) (length stride-parameter)) ;; If same length
                     (list :stride-parameter stride-parameter))
                    ((< (length sprm) (length stride-parameter)) ;; If rtcd's length is shorter than argument's length
                     (list :stride-parameter (subseq stride-parameter 0 (length sprm))))
                    (t ;; If rtcd's length is longer than argument's length
                     (list :stride-parameter (concatenate float-vector stride-parameter (subseq sprm (length stride-parameter))))))
                   ))
             args))))
  (:set-auto-balancer-param
   (&rest args
    &key leg-names
         default-zmp-offsets ;; [mm]
         graspless-manip-arm
         graspless-manip-reference-trans-pos ;; [mm]
         graspless-manip-reference-trans-rot ;; rotation-matrix
         use-force-mode
    &allow-other-keys)
   "Set AutoBalancer param.
    For arguments, please see (send *ri* :get-auto-balancer-param-arguments)."
   (send* self :raw-set-auto-balancer-param
          (append
           (if (and (memq :leg-names args) leg-names)
               (list :leg-names (mapcar #'(lambda (x) (format nil "~A" (string-downcase x))) leg-names)))
           (if (and (memq :default-zmp-offsets args) default-zmp-offsets)
               (let ((dzo (copy-object (send (send self :get-auto-balancer-param) :default_zmp_offsets)))
                     (lnum (length (remove-if-not #'(lambda (x) (send robot x)) '(:rarm :larm :rleg :lleg)))))
                 (setq (dzo . ros::_data)
                       (apply #'concatenate float-vector (mapcar #'(lambda (x) (scale 1e-3 x))
                                                                 (append default-zmp-offsets
                                                                         (make-list (- lnum (length default-zmp-offsets))
                                                                                    :initial-element (float-vector 0 0 0))))))
                 (list :default-zmp-offsets dzo)))
           (if (and (memq :graspless-manip-arm args) graspless-manip-arm)
               (list :graspless-manip-arm (string-downcase graspless-manip-arm)))
           (if (and (memq :graspless-manip-reference-trans-pos args) graspless-manip-reference-trans-pos)
               (list :graspless-manip-reference-trans-pos (scale 1e-3 graspless-manip-reference-trans-pos)))
           (if (and (memq :graspless-manip-reference-trans-rot args) graspless-manip-reference-trans-rot)
               (list :graspless-manip-reference-trans-rot (matrix2quaternion graspless-manip-reference-trans-rot)))
           (if (memq :use-force-mode args)
               (list :use-force-mode
                     (if (or (null use-force-mode) (integerp use-force-mode))
                         use-force-mode
                       (let ((ot (read-from-string (format nil "AUTO_STABILIZER::OPENHRP_AUTOSTABILIZERSERVICE_USEFORCEMODE::*~A*" (substitute (elt "_" 0) (elt "-" 0) (string-downcase use-force-mode))))))
                         (if (boundp ot)
                             (eval ot)
                           (error ";; no such :use-force-mode ~A in :set-auto-balancer-param~%" use-force-mode))
                         ))))
           args)))
  (:eus-footstep->abc-footstep
   (f)
   (instance auto_stabilizer::openhrp_autostabilizerservice_footstep :init
             :pos (scale 1e-3 (send f :worldpos))
             :rot (matrix2quaternion (send f :worldrot))
             :leg (string-downcase (if (find-method f :l/r) (send f :l/r) (send f :name))))
   )
  (:set-st-param
   (&rest args
    &key st-algorithm
         eefm-pos-damping-gain eefm-rot-damping-gain eefm-pos-time-const-support eefm-rot-time-const eefm-swing-pos-spring-gain eefm-swing-pos-time-const eefm-swing-rot-spring-gain eefm-swing-rot-time-const eefm-ee-forcemoment-distribution-weight
    &allow-other-keys)
   "Set Stabilizer parameters.
    For arguments, please see (send *ri* :get-st-param-arguments)."
   (unless (send self :get :default-st-param)
     (send self :put :default-st-param (send self :get-st-param)))
   (let ((prm (send self :get-st-param)))
     (send* self :raw-set-st-param
            (append
             (if (memq :st-algorithm args)
                 (list :st-algorithm
                       (if (or (null st-algorithm) (integerp st-algorithm))
                           st-algorithm
                         (let ((sa (read-from-string (format nil "AUTO_STABILIZER::OPENHRP_AUTOSTABILIZERSERVICE_STALGORITHM::*~A*" (string-downcase st-algorithm)))))
                           (if (boundp sa)
                               (eval sa)
                             (error ";; no such :st-algorithm ~A in :set-st-param~%" st-algorithm))))))
             (if (and (memq :eefm-pos-damping-gain args) eefm-pos-damping-gain)
                 (let ((tmp (send prm :eefm_pos_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-damping-gain))
                   (list :eefm-pos-damping-gain tmp)))
             (if (and (memq :eefm-rot-damping-gain args) eefm-rot-damping-gain)
                 (let ((tmp (send prm :eefm_rot_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-damping-gain))
                   (list :eefm-rot-damping-gain tmp)))
             (if (and (memq :eefm-rot-time-const args) eefm-rot-time-const)
                 (let ((tmp (send prm :eefm_rot_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-time-const))
                   (list :eefm-rot-time-const tmp)))
             (if (and (memq :eefm-pos-time-const-support args) eefm-pos-time-const-support)
                 (let ((tmp (send prm :eefm_pos_time_const_support)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-time-const-support))
                   (list :eefm-pos-time-const-support tmp)))
             (if (and (memq :eefm-swing-pos-spring-gain args) eefm-swing-pos-spring-gain)
                 (let ((tmp (send prm :eefm_swing_pos_spring_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-pos-spring-gain))
                   (list :eefm-swing-pos-spring-gain tmp)))
             (if (and (memq :eefm-swing-pos-time-const args) eefm-swing-pos-time-const)
                 (let ((tmp (send prm :eefm_swing_pos_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-pos-time-const))
                   (list :eefm-swing-pos-time-const tmp)))
             (if (and (memq :eefm-swing-rot-spring-gain args) eefm-swing-rot-spring-gain)
                 (let ((tmp (send prm :eefm_swing_rot_spring_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-rot-spring-gain))
                   (list :eefm-swing-rot-spring-gain tmp)))
             (if (and (memq :eefm-swing-rot-time-const args) eefm-swing-rot-time-const)
                 (let ((tmp (send prm :eefm_swing_rot_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-rot-time-const))
                   (list :eefm-swing-rot-time-const tmp)))
             (if (and (memq :eefm-ee-forcemoment-distribution-weight args) eefm-ee-forcemoment-distribution-weight)
                 (let ((tmp (send prm :eefm_ee_forcemoment_distribution_weight)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-ee-forcemoment-distribution-weight))
                   (list :eefm-ee-forcemoment-distribution-weight tmp)))
             args))))
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


