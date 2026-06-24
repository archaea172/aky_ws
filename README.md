# aky_ws

ROS 2 Humble を前提にした群制御・シミュレーション用ワークスペースです。現状は、Gazebo 上でロボット群と知能ロボコン風ステージを起動し、boid/follower 系の制御ノード、BehaviorTree によるライフサイクル操作、Leader 経路生成、MuJoCo での強化学習環境を扱う構成になっています。

## 確認状況

調査日: 2026-05-28

確認できたこと:

- `colcon build --symlink-install` は全10パッケージで成功。
- `source install/setup.zsh` 後、`inrof_swerm` の Python バインディングは import 可能。
- `inrof_mujoco` の `SwermEnv.reset()` / `SwermEnv.step()` は、ローカルの未コミット変更込みでは GUI なしで実行可能。

未確認・要確認:

- `colcon test` は完走するものの、5パッケージで lint/style 失敗があります。主な内容は `uncrustify` の整形差分と Python launch ファイルの `flake8` 指摘です。
- Gazebo GUI を伴う launch は、この調査では起動確認していません。
- MuJoCo GUI viewer は、この調査ではウィンドウ起動確認していません。
- `src/inrof_mujoco/src/inrof_mujoco/rl_test.py` と `src/inrof_mujoco/scripts/view_rl_env.py` に未コミットのローカル変更があります。docs-only PR に含めるか、別PRに分けるか確認が必要です。
- target branch の現行 `rl_test.py` は import 時に PPO 学習を開始する形のため、ライブラリとして import する使い方はコード修正後に明記するのが安全です。

## パッケージ概要

| パッケージ | 役割 |
| --- | --- |
| `inrof_core` | 複数パッケージをまとめて起動する launch を持つ入口パッケージ。 |
| `inrof_sim` | Gazebo で spawn する robot/leader の URDF。 |
| `irc_table` | 知能ロボコン風ステージ、ボールモデル、ランダムボール spawn launch。 |
| `swerm_msgs` | Leader 移動、Lifecycle 操作、LeaderPath 生成用の action/service 定義。 |
| `inrof_swerm` | Boid/Follower/Sheep のコア制御、ROS ノード、Python バインディング。 |
| `inrof_control` | Leader の経路生成 service と Leader 位置 publish action server。 |
| `inrof_bt` | BehaviorTree.CPP/ROS2 を使った lifecycle と leader_pos action の実行ノード。 |
| `BehaviorTree.ROS2` | BehaviorTree.CPP と ROS 2 を接続する外部ライブラリ。 |
| `inrof_mujoco` | MuJoCo + Gymnasium + Stable-Baselines3 による強化学習環境。 |

## セットアップとビルド

ROS 2 環境を source したうえで、ワークスペース root から実行します。

```bash
colcon build --symlink-install
source install/setup.zsh
```

今回の確認では `colcon build --symlink-install` が成功しています。

テストは以下で実行できます。

```bash
colcon test
colcon test-result --verbose
```

現状の `colcon test-result --verbose` では、`inrof_core`, `inrof_control`, `inrof_bt`, `inrof_swerm`, `irc_table` に lint/style 失敗があります。動作ロジックのテスト失敗というより、整形・未使用 import・行長・末尾空白などの指摘が中心です。

## 主な起動方法

### Gazebo シミュレーション

知能ロボコン風ステージ、ボール、5台のロボット、Gazebo-ROS bridge を起動する入口です。

```bash
source install/setup.zsh
ros2 launch inrof_core sim.launch.py
```

`sim.launch.py` は以下をまとめて起動します。

- `irc_table/worlds/irc_table.sdf`
- `irc_table/launch/spawn_random_balls.launch.py`
- `inrof_sim/urdf/robot.urdf` を使った `robot_0` から `robot_4` の spawn
- `/robot_N/cmd_vel`, `/robot_N/odometry`, `/tf` まわりの bridge

ステージとボールだけを確認したい場合は、`irc_table` 単体でも起動できます。

```bash
source install/setup.zsh
ros2 launch irc_table irc_table.launch.py
```

### Follower + BehaviorTree

Follower 制御、map server、Leader 経路生成、Lifecycle 操作、BehaviorTree 実行をまとめる入口です。

```bash
source install/setup.zsh
ros2 launch inrof_core follower_bt.launch.py
```

`follower_bt.launch.py` は以下を起動します。

- `inrof_swerm/follower_node`
- `nav2_map_server/map_server`
- `inrof_control/leader_pos_server`
- `inrof_control/path_generate_server`
- `inrof_bt/lifecycle_server`
- `inrof_bt/inrof_bt_node`

BehaviorTree は `src/inrof_bt/config/main_bt.xml` を読み込み、`follower_node` を inactive -> active に遷移させ、`leader_pos` action で Leader を移動し、最後に inactive に戻す流れです。

### 個別ノード

個別に動かす場合の主な executable は以下です。

```bash
source install/setup.zsh

ros2 run inrof_swerm boid_node --ros-args --params-file src/inrof_swerm/config/boid.yaml
ros2 run inrof_swerm follower_node --ros-args --params-file src/inrof_swerm/config/follower.yaml
ros2 run inrof_swerm sheep_node --ros-args --params-file src/inrof_swerm/config/sheep.yaml

ros2 run inrof_control path_generate_server
ros2 run inrof_control leader_pos_server

ros2 run inrof_bt lifecycle_server
ros2 run inrof_bt inrof_bt_node
```

`follower_node` は lifecycle node です。単体起動時は map を受け取り、configure/activate されてから制御 publish を始めます。

## ROS インターフェース

主な topic/action/service は以下です。

| 名前 | 種別 | 提供元 | 概要 |
| --- | --- | --- | --- |
| `robot_N/odometry` | topic subscribe | `inrof_swerm` | 各ロボットの位置・速度入力。 |
| `robot_N/cmd_vel` | topic publish | `inrof_swerm` | 各ロボットへの速度指令。 |
| `leader/odometry` | topic publish/subscribe | `inrof_control`, `inrof_swerm` | Leader の位置入力。 |
| `leader_path` | topic publish | `path_generate_server` | 生成した Leader 経路。 |
| `leader_path_service` | service | `path_generate_server` | start/goal/resolution から `nav_msgs/Path` を返す。 |
| `leader_pos` | action | `leader_pos_server` | Leader を start から goal へ移動させる。 |
| `lifecycle_server` | action | `lifecycle_server` | lifecycle node の状態遷移を action として扱う。 |

## inrof_swerm の使いどころ

`inrof_swerm` は C++ コアと ROS ノードの両方を持っています。

- `BoidCore`: 分離、整列、凝集、壁回避による群速度を計算。
- `FollowerCore`: Boid の力に Leader 追従力を加える。
- `SheepCore`: Boid の力に dog/leader から逃げる力を加える。
- Python binding: `BoidPrams`, `BoidCore`, `FollowerCore`, `SheepCore` を Python から利用可能。

Python binding の import 確認例:

```bash
source install/setup.zsh
python3 -c "import inrof_swerm; print(inrof_swerm.FollowerCore)"
```

## inrof_mujoco の注意

`src/inrof_mujoco` は `uv` 管理の Python パッケージで、MuJoCo/Gymnasium/Stable-Baselines3 を使う構成です。

```bash
cd src/inrof_mujoco
uv run python src/inrof_mujoco/rl_test.py
```

ただし、target branch の現行 `rl_test.py` は PPO 学習をすぐ開始するため、短時間の動作確認用コマンドとしては重いです。GUI デバッグや import-safe な利用手順は、現在のローカル未コミット変更をどう扱うか確認したうえでREADMEに追記するのが安全です。

`scripts/export_model.py` は `models/boid.xml` を生成して MuJoCo viewer を開くスクリプトですが、既存モデルを書き換えるため、実行前に差分を確認してください。
