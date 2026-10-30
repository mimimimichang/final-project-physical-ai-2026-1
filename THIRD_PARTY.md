# 第三者著作物の出典とライセンス

この文書では、同梱した第三者著作物の出典、ライセンス、変更内容を示す。セットアップと操作方法は[README.md](README.md)を参照。

## 出典

- 上流リポジトリ：[OsawaKousei/2DRoboPrac_ws](https://github.com/OsawaKousei/2DRoboPrac_ws)
- 取得したコミット：`123fd2e4ddfccf420499a02a3852c17a369f93bb`
- 元の場所：`gz_ws/MoveTheRobot/models/`
- ライセンス：Apache License 2.0。全文を[licenses/2DRoboPrac_ws-Apache-2.0.txt](licenses/2DRoboPrac_ws-Apache-2.0.txt)に同梱している。
- 参考記事：[GazeboとROS 2を連携してロボットを操作する](https://qiita.com/N622/items/9d89f77d85d9da0af29e)

確認した範囲では、上流リポジトリと対象モデルのディレクトリに別のNOTICEファイルは含まれていなかった。無関係なROS・マイコン用ディレクトリは取り込んでいない。

## 同梱した上流ファイル

| ファイル | 出典と扱い |
|---|---|
| [models/world.sdf](models/world.sdf) | 上流ワールドをGazebo Harmonicに対応させ、前方LiDARと静止コーン2個を追加 |
| [models/meshes/2DRoboPrac.stl](models/meshes/2DRoboPrac.stl) | ロボットの衝突形状。上流から変更せず複製 |
| [models/meshes/2DRoboPrac.dae](models/meshes/2DRoboPrac.dae) | ロボットの表示形状。上流から変更せず複製 |

## 上流ワールドへの変更

- Gazeboプラグインのファイル名をIgnition Gazebo形式からGazebo Sim形式へ変更。
- プラグインのクラス名を`ignition::gazebo::systems::*`から`gz::sim::systems::*`へ変更。
- キー入力と速度指令に使うメッセージ型を`ignition.msgs.*`から`gz.msgs.*`へ変更。
- 前方LiDARと静止コーン2個を追加し、変更内容をSDF冒頭に記載。

ロボットの形状、質量、車輪寸法、ジョイント構成は変更していない。キーボード操作用のプラグインも引き継いでいる。

同梱したライセンスは上流由来のファイルに適用されるもので、このプロジェクトで追加したコードのライセンスを定めるものではない。