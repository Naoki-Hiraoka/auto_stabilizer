```shell
rtmlaunch auto_stabilizer_choreonoid_sample jaxon_jvrc_choreonoid.launch
```

```lisp
roseus
(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "package://auto_stabilizer_choreonoid_sample/euslisp/auto-stabilizer-interface.l")
(jaxon_jvrc-init)
(setq *robot* *jaxon_jvrc*)
;;(send *ri* :start-auto-balancer)
;;(send *ri* :start-st)

(send *ri* :go-velocity 0 0 0)
(send *ri* :go-stop)
(send *ri* :go-pos 1 0 0)
(send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -190 0) :name :rleg)
              (make-coords :pos (float-vector 0 190 0) :name :lleg)))
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 -190 0) :name :rleg)
              (make-coords :pos (float-vector 0 190 0) :name :lleg))
        (list 70 70)
        (list 1.0 1.0))

(send *ri* :start-impedance :arms)
(send *ri* :stop-impedance :arms)

(print-ros-msg (send *ri* :get-auto-stabilizer-param))

;; stair
(send *robot* :reset-pose)
(send *robot* :legs :move-end-pos #F(0 0 150))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 0 90 0) :name :lleg)
              (make-coords :pos (float-vector 300 -90 200) :name :rleg)
              (make-coords :pos (float-vector 300 90 200) :name :lleg)
              (make-coords :pos (float-vector 600 -90 400) :name :rleg)
              (make-coords :pos (float-vector 600 90 400) :name :lleg)
              (make-coords :pos (float-vector 900 -90 600) :name :rleg))
        (list 70 70 70 70 70 70 70)
        (list 1.4 1.4 1.4 1.4 1.4 1.4 1.4))
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 300 90 200) :name :lleg)
              (make-coords :pos (float-vector 600 -90 400) :name :rleg)
              (make-coords :pos (float-vector 900 90 600) :name :lleg)
              (make-coords :pos (float-vector 1200 -90 800) :name :rleg))
        (list 50 50 50 50 50)
        (list 2.0 2.0 2.0 2.0 2.0))
```