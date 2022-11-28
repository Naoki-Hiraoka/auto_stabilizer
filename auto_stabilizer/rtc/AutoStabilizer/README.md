## ドキュメント

https://docs.google.com/presentation/d/1oJWIUqfYPsNI-mGdanSZhfvkEQ5jZHm7uG4CLNwVyM8/edit?usp=sharing (JSK internal)

## 妥協点メモ

- コメントは英語が望ましいが、理解の容易さと、英語を書く手間を嫌ってコメントを書かなくなったら本末転倒であることを考え日本語で妥協した
- idlのmodule名は、パッケージ名とそろえてauto_stabilizerにしたかった。しかし、hrpsys-baseのhrpsys_config.pyが、module名がOpenHRPでないとやりにくい書かれ方になっている. hrpsys-baseのhrpsys_config.pyを使わないという方法もあるが変更が大きくなるので、仕方なくmodule名をOpenHRPにした
- port名はref*, act*, gen* という命名規則にしている. しかし、port "qRef", "q"は、hrpsys-baseのhrpsys_config.pyの作法に揃えるため、やむを得ず例外的にこの名前にしている.
- AutoStabilizer::endEffectors_は、可変長にしたり順番を任意にしたりといろいろ拡張性をもたせることも考えられるが、masterのhrpsysが中途半端に拡張性をもたせた結果その機能を十分に使いこなせていないうえに可読性を下げる結果にしかなっていないので、今回は単純に順番固定の配列にして、0番目が右脚、1番目が左脚という仮定も堂々とおくことにした。
- dtは、本来はonExecute中で`1.0 / this->get_context(ec_id)->get_rate()`とすれば与えずとも自動で計算できるのだが、choreonoidのBodyRTCItemを使う場合には正しく計算できない.choreonoidのBodyRTCItemを使わないという方法もあるが、変更が大きくなるのでここでは簡単のため妥協してconfファイルからdtまたはexec_cxt.periodic.rateを与える形にした
- 経験上、良く整理されたオブジェクト指向プログラミングはとても読みやすいし改造しやすい。一方で、よく整理されていないオブジェクト指向プログラミングはとても読みにくいし改造しにくい。良く整理してオブジェクト指向プログラミングをするのが望ましいが、よほどの良いアイデアが無い限り難しいため、不用意にこれを志向すると、結局よく整理されていないオブジェクト指向プログラミングになってしまう危険性が高い。そのため、良く整理するのが難しいところは最初から妥協して、宣言型プログラミングを指向した。
- footStepが90度以上傾くと破綻するアルゴリズムも気にせず採用. 支持点が重心より高い位置にあると破綻するアルゴリズムも気にせず採用. これらにこだわりたければ多点接触でやる
- ServoGainPercentageは、[abs](https://github.com/kindsenior/hrpsys-base/blob/643fc56d2d1a39b57f00d5e38368f90f25045266/rtc/RobotHardware/RobotHardware.cpp#L54)のようにRobotHardwareのportから与える方式の方が扱いやすい. さらに言えば、SequencePlayerが補間して出す方式にした方が扱いやすい. しかし、変更を小さくするためにひとまずRobotHardwareのserviceから与える方式にした.

## FootGuidedControllerの導出

```
Y. Kakiuchi & Y. Kojio & N. Imaoka & D. Kusuyama & S. Sato & Y. Matsuura & T. Ando & M. Inaba (2022) 
Trajectory Generation and Compensation for External Forces with a Leg-Wheeled Robot Designed for Human Passengers, 
2022 IEEE-RAS 21st International Conference on Humanoid Robots (Humanoids)
```
```
T. Yamamoto & T. Sugihara (2020) Foot-guided control of a biped robot through ZMP manipulation, 
Advanced Robotics, 34:21-22, 1472-1489, DOI: 10.1080/01691864.2020.1827031
```

### 問題定義

Capture Pointを$x$、重心を$c$、ZMPを$z$と表すと、
$$x = c + \sqrt{\frac{h}{g}}\dot{c}$$
$$\ddot{c} = \frac{g}{h}(c-z)$$
であるから、

$$
\begin{eqnarray}
  \dot{x} &=& \dot{c} + \sqrt{\frac{h}{g}}\ddot{c} \\
  &=& \dot{c} + \sqrt{\frac{g}{h}}(c-z) \\
  &=& \sqrt{\frac{g}{h}}(x-z) \\
\end{eqnarray}
$$

がなりたつ. これを参考にして、状態方程式

$$\dot{x} = w(x-u-l)$$

($w$, $l$はconst. 例えば$w=\sqrt{\frac{g}{h}}$, $l=h$)が成り立つような系において、次のような最適制御問題を考える.

$$ u(t) = \mathrm{argmin} \frac{1}{2} \int^{T_n}_{T_0} (u(t) - u^r(t))^2 dt $$

$$
\begin{eqnarray}
  \mathrm{s.t.} && \dot{x} = w(x-u-l) \\
  && x(T_0) = x_0\\
  && x(T_n) = x_f
\end{eqnarray}
$$

$$
\begin{eqnarray}
  \mathrm{where} & u^r = \left\lbrace
  \begin{array}{cc}
  u^r_1 = a_1t+b_1 & (t\leq T_1)\\
  u^r_2 = a_2t+b_2 & (T_1 < t\leq T_2)\\
  \vdots\\
  u^r_n = a_nt+b_n & (T_{n-1} < t\leq T_{n})
  \end{array}\right.
\end{eqnarray}
$$

### 変分法
下記文献の5.2節に解説されている変分法を使用して解く.

大塚 敏之, アドバンスト制御のための変分法と最適制御 -初学者のために- (第1回), 計測と制御, 2006, 45 巻, 10 号, p. 899-907, 公開日 2009/11/26, Online ISSN 1883-8170, Print ISSN 0453-4662, https://doi.org/10.11499/sicejl1962.45.899, https://www.jstage.jst.go.jp/article/sicejl1962/45/10/45_10_899/_article/-char/ja

文献では、目的関数
$$\min J = \eta(x(t_0),t_0) + \varphi(x(t_f),t_f) +\int^{t_f}_{t_0}L(x(t),u(t),t)dt$$

状態方程式、等式拘束条件、初期条件、終端条件

$$
\begin{eqnarray}
  \dot{x}(t) = f(x(t),u(t),t)\\
  C(x(t),u(t),t)=0\\
  X(x(t_0),t_0)=0\\
  \psi(x(t_f),t_f)=0\\
\end{eqnarray}
$$

が与えられたときの解法が紹介されている.

今回は

$$
\begin{eqnarray}
  \eta = 0\\
  \varphi = 0\\
  C=0
\end{eqnarray}
$$

なので一部簡単になる.

ハミルトニアンは

$$H(x,u,\lambda,\rho,t) = L(x,u,t)+\lambda^Tf(x,u,t)$$

となり、汎関数の停留条件は

$$
\begin{eqnarray}
  \dot\lambda = - \left(\frac{\partial{H}}{\partial{x}}\right)^T,\\
  \frac{\partial H}{\partial u}(x,u,\lambda,\rho,t)=0
\end{eqnarray}
$$

である.

### 求解

ハミルトニアンは

$$ H = \frac{1}{2}(u-u^r)^2+\lambda w(x-u-l)$$

であるから、

$$\dot\lambda = - \left(\frac{\partial{H}}{\partial{x}}\right)^T$$

に代入し、

$$
\begin{eqnarray}
  \dot\lambda = - \lambda w\\
  \therefore \lambda = C e^{-wt}
\end{eqnarray}
$$

これを

$$\frac{\partial H}{\partial u}(x,u,\lambda,\rho,t)=0$$

に代入し、

$$
\begin{eqnarray}
  u - u^r - \lambda w &=& 0\\
  u &=& u^r + Cwe^{-wt}
\end{eqnarray}
$$

これを状態方程式

$$\dot{x} = w(x-u-l)$$

に代入し、

$$
\begin{eqnarray}
  \dot{x} &=& w(x-u^r - Cwe^{-wt}-l)\\
  \dot{x}_i &=& wx_i-wa_it - wb_i - Cw^2e^{-wt}-wl
\end{eqnarray}
$$

を得る.非斉次微分方程式の解法を用いてこれを解く.

まず、斉次方程式

$$\dot{x}_i = wx$$

の一般解は、

$$x_i = D_i e^{wt}$$

である.次に、非斉次方程式

$$\dot{x}_i = wx_i-wa_it - wb_i - Cw^2e^{-wt}-wl$$

の特解は、

$$x_i = \frac{a_i}{w}+a_i t + b_i + \frac{1}{2}wCe^{-wt}+l$$

である.よって、上記非斉次微分方程式の一般解は、

$$x_i = D_i e^{wt} + \frac{a_i}{w}+a_i t + b_i + \frac{1}{2}wCe^{-wt}+l$$

である.

連続条件より、

$$x_{i+1}(T_i) = x_i(T_i)$$

であるから、

$$
\begin{eqnarray}
  D_{i+1} e^{wT_i} + \frac{a_{i+1}}{w}+a_{i+1} T_i + b_{i+1} + \frac{1}{2}wCe^{-wT_i}+l\\
  = D_i e^{wT_i} + \frac{a_i}{w}+a_i T_i + b_i + \frac{1}{2}wCe^{-wT_i}+l\\
  \Leftrightarrow (D_{i+1}-D_i) e^{wT_i} = (a_i-a_{i+1})(\frac{1}{w}+T_i)+(b_i-b_{i+1})\\
  \Leftrightarrow (D_{i+1}-D_i) e^{wT_i} = u^r_i(T_i)-u^r_{i+1}(T_i)+\frac{a_i-a_{i+1}}{w}\\
  \Leftrightarrow D_i = D_1 + \sum_{j=1}^{i-1}e^{-wT_j}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w} \right]
\end{eqnarray}
$$

始点より、

$$x(T_0)=x_0$$

であるから、

$$
\begin{eqnarray}
  x_0 &=& D_1 e^{wT_0} + \frac{a_1}{w}+a_1 T_0 + b_1 + \frac{1}{2}wCe^{-wT_0}+l\\
  D_1 &=& e^{-wT_0}(x_0-b_1-\frac{a_1}{w}-a_1T_0-l)-\frac{1}{2}wCe^{-2wT_0}
\end{eqnarray}
$$

よって、

$$
\begin{eqnarray}
  D_i = e^{-wT_0}(x_0-b_1-\frac{a_1}{w}-a_1T_0-l)-\frac{1}{2}wCe^{-2wT_0} +\sum_{j=1}^{i-1}e^{-wT_j}\left [ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w} \right]
\end{eqnarray}
$$

終点より

$$x(T_n)=x_f$$

であるから、

$$
\begin{eqnarray}
  x_f &=& D_n e^{wT_n} + \frac{a_n}{w}+a_n T_n + b_n + \frac{1}{2}wCe^{-wT_n}+l \\
  D_n &=& e^{-wT_n}(x_f-b_n-\frac{a_n}{w}-a_nT_n-l)-\frac{1}{2}wCe^{-2wT_n}
\end{eqnarray}
$$

これを$D_i=$の式に代入し、($b_0=x_0-l$,$b_{n+1}=x_f-l$,$a_0=0$,$a_{n+1}=0$とおく)

$$
\begin{eqnarray}
  \frac{1}{2}wC(e^{-2wT_0}-e^{-2wT_n})=\sum^n_{j=0} e^{-wT_j}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w}\right]\\
  C = \frac{2}{w(e^{-2wT_0}-e^{-2wT_n})}\sum^n_{j=0} e^{-wT_j}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w}\right]\\
  C = \frac{2e^{wT_0}}{w(1-e^{-2w(T_n-T_0)})}\sum^n_{j=0} e^{-w(T_j-T_0)}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w}\right]
\end{eqnarray}
$$

これを$u = u^r + Cwe^{-wt}$に代入し、

$$
\begin{eqnarray}
  u(t) = u^r(t)+\frac{2e^{-w(t-T_0)}}{(1-e^{-2w(T_n-T_0)})}\sum^n_{j=0} e^{-w(T_j-T_0)}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w}\right]
\end{eqnarray}
$$

以上より、

$$u(T_0) = u^r(T_0)+\frac{2}{(1-e^{-2w(T_n-T_0)})}\sum^n_{j=0} e^{-w(T_j-T_0)}\left[ u^r_j(T_j)-u^r_{j+1}(T_j)+\frac{a_j-a_{j+1}}{w}\right]$$

(ただし$b_0=x_0-l$,$b_{n+1}=x_f-l$,$a_0=0$,$a_{n+1}=0$)
