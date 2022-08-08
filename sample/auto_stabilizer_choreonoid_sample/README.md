```shell
rtmlaunch auto_stabilizer_choreonoid_sample jaxon_jvrc_choreonoid.launch
```

```lisp
roseus
(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "package://auto_stabilizer_choreonoid_sample/euslisp/auto-stabilizer-interface.l")
(jaxon_jvrc-init)
;;(send *ri* :start-auto-balancer)
;;(send *ri* :start-st)
(send *ri* :go-velocity 0 0 0)
```