## 環境構築
### rosinstall
```bash
catkin_ws/src$ emacs -nw .rosinstall # auto_stabilizerのディレクトリにある.rosinstall と auto_stabilizer_choreonoid_sampleのディレクトリにある.rosinstallの内容を記入する. このリポジトリが入っていないので追加する.
catkin_ws/src$ wstool update
```
### ビルド
```bash
catkin_ws/src$ cd ..
# 依存ライブラリをインストールする.
catkin_ws$ rosdep install -r --from-paths src --ignore-src -y
# 参考 https://github.com/start-jsk/rtmros_hironx/issues/539
catkin_ws$ echo "export ORBgiopMaxMsgSize=2097152000" >> ~/.bashrc
catkin_ws$ source ~/.bashrc
# for choreonoid. 参考 https://github.com/start-jsk/rtmros_choreonoid
# choreonoidはrosdepに対応しておらず，代わりに依存ライブラリをaptでインストールするスクリプトがあるのでそれを実行
catkin_ws$ ./src/choreonoid/misc/script/install-requisites-ubuntu-18.04.sh # if ubuntu 18.04
# choreonoidのコンパイルオプションを設定
catkin_ws$ patch -p1 -d src/choreonoid < src/rtm-ros-robotics/rtmros_choreonoid/choreonoid.patch
```
* ビルド
```bash
catkin_ws$ source /opt/ros/melodic/setup.bash
catkin_ws$ catkin build auto_stabilizer_choreonoid_sample # 失敗してもそのままもう一回同じビルドを行うと通ることがある
catkin_ws$ source devel/setup.bash
```

## 実行

```shell
rtmlaunch auto_stabilizer_choreonoid_sample jaxon_jvrc_choreonoid.launch

# rtmlaunch auto_stabilizer_choreonoid_sample jaxon_jvrc_choreonoid.launch ENVIRONMENT_YAML:=`rospack find vnoid_world`/sample/sample_field.yaml LOAD_OBJECTS:=true
```

```lisp
roseus
(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_jvrc-init)
(setq *robot* *jaxon_jvrc*)
;;(send *ri* :start-auto-balancer)
;;(send *ri* :start-st)

(send *ri* :set-auto-stabilizer-param :goal-offset -0.05)

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
(send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -100 0) :name :rleg)
              (make-coords :pos (float-vector 0 100 0) :name :lleg)
              (make-coords :pos (float-vector 0 100 0) :name :lleg)
              (make-coords :pos (float-vector 0 100 0) :name :lleg)))
(send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -100 0) :name :rleg)
              (make-coords :pos (float-vector 0 100 0) :rpy (float-vector pi/2 0 0) :name :lleg)))
(send *ri* :start-impedance :arms)
(send *ri* :stop-impedance :arms)
(print-ros-msg (send *ri* :get-auto-stabilizer-param))

;; 片足立ちしてもう片方の足をangle-vectorで動かす.
(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #F(0 100 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :set-auto-stabilizer-param
      :reference-frame (list nil t)
      :is-hand-fix-mode t
      :default-zmp-offsets (list #F(0 0) #F(0 0))
      )
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 100 0) :name :lleg)
        (make-coords :pos (float-vector 0 -100 50) :name :rleg))
        (list 70 70)
        (list 1.0 1.0) (list nil t))
(send *robot* :rleg :move-end-pos #F(0 0 50))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :wait-foot-steps) ;; 歩行中は:is-manual-control-mode を変えることができない仕様になっているので
(send *ri* :set-auto-stabilizer-param
      :is-manual-control-mode (list t nil)
      )
(send *robot* :rleg :move-end-pos #F(0 -50 50))
(send *robot* :rleg :move-end-rot 15 :y :local)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :go-pos 0 0 0)
(send *robot* :reset-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :set-auto-stabilizer-param
      :reference-frame (list t t)
      :is-hand-fix-mode nil
      :default-zmp-offsets (list #F(0 0.02) #F(0 -0.02))
      )

;; stair. 階段の前に移動させてから行う
(send *robot* :reset-pose)
(send *robot* :legs :move-end-pos #F(0 0 150))
(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :set-auto-stabilizer-param
      :goal-offset 0.0
      :default-double-support-ratio 0.2
      :overwritable-min-time 1.4
      )
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 100 0) :name :lleg)
              (make-coords :pos (float-vector 300 -100 200) :name :rleg)
              (make-coords :pos (float-vector 300 100 200) :name :lleg)
              (make-coords :pos (float-vector 600 -100 400) :name :rleg)
              (make-coords :pos (float-vector 600 100 400) :name :lleg)
              (make-coords :pos (float-vector 900 -100 600) :name :rleg)
              (make-coords :pos (float-vector 900 100 600) :name :lleg)
              )
        (list 70 70 70 70 70 70 70)
        (list 1.4 1.4 1.4 1.4 1.4 1.4 1.4))
(send *ri* :set-auto-stabilizer-param
      :goal-offset 0.0
      :default-double-support-ratio 0.2
      :overwritable-min-time 2.0
      )
(send *ri* :set-foot-steps-with-param
        (list (make-coords :pos (float-vector 0 -100 0) :name :rleg)
              (make-coords :pos (float-vector 300 100 200) :name :lleg)
              (make-coords :pos (float-vector 600 -100 400) :name :rleg)
              (make-coords :pos (float-vector 900 100 600) :name :lleg)
              (make-coords :pos (float-vector 1200 -100 800) :name :rleg)
              (make-coords :pos (float-vector 1200 100 800) :name :lleg)
              )
        (list 50 50 50 50 50 50)
        (list 2.0 2.0 2.0 2.0 2.0 2.0))
```

### memo

シミュレーションでは、次のようなパラメータにした方が性能が良い
```
(send *ri* :set-auto-stabilizer-param
           :footguided-balance-time 0.4
           :overwritable-min-step-time 0.5
           )
```

視覚無しで傾斜のある地面を歩くときは、次のようなパラメータにした方が性能が良い. とはいえ視覚を使うのが正解.
```
(send *ri* :set-auto-stabilizer-param
           :contact-detection-threshold 200
           :emergency-step-cp-check-margin 0.1
           :goal-offset -0.05
           )
```

```
(send *ri* :set-auto-stabilizer-param :contact-detection-threshold 300.0)
;;(send *ri* :set-auto-stabilizer-param :is-stable-go-stop-mode nil)
(send *robot* :reset-pose)
(send *robot* :legs :move-end-pos #F(0 0 150))
(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
```
