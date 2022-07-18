## 妥協メモ

- コメントは英語にしたかったが、理解の容易さを考え日本語で妥協した
- idlのmodule名は、パッケージ名とそろえてauto_stabilizerにしたかった。しかし、hrpsys-baseのhrpsys_config.pyが、module名がOpenHRPでないとやりにくい書かれ方になっている. hrpsys-baseのhrpsys_config.pyという方法もあるが変更が大きくなるので、仕方なくmodule名をOpenHRPにした
- port名はref*, act*, gen* という命名規則にしている. しかし、port "qRef", "q"は、hrpsys-baseのhrpsys_config.pyの作法に揃えるため、やむを得ず例外的にこの名前にしている.
- AutoStabilizer::endEffectors_は、可変長にしたり順番を任意にしたりといろいろ拡張性をもたせることも考えられるが、masterのhrpsysが中途半端に拡張性をもたせた結果その機能を十分に使いこなせていないうえに可読性を下げる結果にしかなっていないので、今回は単純に順番固定の配列にして、0番目が右脚、1番目が左脚という仮定も堂々とおくことにした。
- dtは、本来はonExecute中で`1.0 / this->get_context(ec_id)->get_rate()`とすれば与えずとも自動で計算できるのだが、choreonoidのBodyRTCItemを使う場合には正しく計算できない.choreonoidのBodyRTCItemを使わないという方法もあるが、変更が大きくなるのでここでは簡単のため妥協してconfファイルからdtまたはexec_cxt.periodic.rateを与える形にした