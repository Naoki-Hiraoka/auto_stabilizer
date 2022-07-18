```shell
rtmlaunch auto_stabilizer_choreonoid_sample jaxon_jvrc_choreonoid.launch
```

```lisp
roseus
(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "./auto-stabilizer-interface.l")
(jaxon_jvrc-init)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
```