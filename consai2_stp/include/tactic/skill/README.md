# Skillの作成方法
## Skillクラスを継承
オーバーライドする必要のある関数は以下の通り．
CLionユーザはCtrl+Oで簡単にオーバーライド関数が書ける．
~~~c++
virtual Status run(const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder)
~~~
この関数が毎フレーム実行され，Status型で状態を返す．
## 構築関数
構築関数はstaticなので仮想関数ではない．
従って自前で作る必要がある．引数はコンストラクタの引数をそのまま入れる
~~~c++
static std::shared_ptr<Component> build(...)
~~~
## 実装予定Skill
- receive pass
- pass to point
- dribble turn
