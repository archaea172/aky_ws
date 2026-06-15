# TF 利用箇所と注意点

最終更新: 2026-06-15

このメモは、このワークスペース内で ROS の TF/tf2 を使っている箇所と、現状のフレーム設計で気になる点をまとめたものです。TensorFlow の `tf` 利用は見つかっていません。

## TF を使っている箇所

| ファイル | 内容 |
| --- | --- |
| `src/inrof_core/launch/follower_bt.launch.py` | `tf2_ros/static_transform_publisher` で `map -> odom` の静的TFを配信する。 |
| `src/inrof_core/launch/sim.launch.py` | 各 `robot_i` に対して `robot_state_publisher` を起動し、`frame_prefix=robot_i/` を付ける。Gazebo の `/model/<robot>/pose` も `/tf` に bridge する。 |
| `src/inrof_core/launch/rl_sim.launch.py` | `sim.launch.py` と同様に、複数ロボットの `robot_state_publisher` と Gazebo pose の `/tf` bridge を起動する。 |
| `src/inrof_sim/urdf/robot.urdf` | Gazebo `OdometryPublisher` plugin を使い、`odom_frame` を `odom` にする。 |
| `src/inrof_sim/urdf/leader.urdf` | Gazebo `OdometryPublisher` plugin を使い、`robot_base_frame=leader/base_footprint` と `tf_topic=/model/leader/tf` を指定する。ただし、この URDF を起動している launch は現状見当たらない。 |
| `src/inrof_sim/package.xml` | `tf2_msgs` と `tf2_ros` を実行時依存として宣言している。 |

## TF を直接使っていないが座標系に関係する箇所

`inrof_swerm` と `inrof_control` の主要ノードは、TF buffer や `lookupTransform` は使っていません。代わりに `nav_msgs/Odometry` の位置・速度をそのまま同一座標系の値として扱っています。

主な例は次の通りです。

- `src/inrof_swerm/src/ros/follower_node.cpp`: `robot_i/odometry` と `leader/odometry` を購読し、座標変換せず制御計算に使う。
- `src/inrof_swerm/src/ros/boid_node.cpp`: `robot_i/odometry` の pose/twist をそのまま Boid 計算に使う。
- `src/inrof_swerm/src/ros/sheep_node.cpp`: `robot_i/odometry` と `leader/odometry` を同一座標系として扱う。
- `src/inrof_control/src/leader_pos_server.cpp`: `leader/odometry` を publish する。
- `src/inrof_control/src/inrof_control/leader_rl.py`: `leader/odometry` を publish し、`robot_i/odometry` を観測として使う。

この設計では、Odometry の `header.frame_id` と TF ツリーがずれてもコード内では検出されません。シミュレーション上で全 odometry が同じワールド座標として出ていることが前提です。

## 怪しい点

### `inrof_core` の依存関係が不足している

`follower_bt.launch.py`, `sim.launch.py`, `rl_sim.launch.py` は `tf2_ros`, `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim` などを起動しますが、`src/inrof_core/package.xml` にはこれらの `exec_depend` がありません。launch を提供するのは `inrof_core` なので、実行時依存も `inrof_core` に宣言するのが妥当です。

### `robot.urdf` の base frame が明示されていない

`src/inrof_sim/urdf/robot.urdf` の `OdometryPublisher` plugin は `odom_frame` だけを指定しています。複数ロボットを spawn する構成では、Odometry/TF の child frame が `robot_i/base_footprint` になる保証を確認した方がよいです。`leader.urdf` のように `robot_base_frame` を明示する方が安全です。

### Gazebo pose bridge と `robot_state_publisher` のフレームがつながるか未確認

`sim.launch.py` と `rl_sim.launch.py` は Gazebo の `/model/<robot>/pose` を `/tf` に bridge しています。一方、`robot_state_publisher` は `frame_prefix=robot_i/` により `robot_i/base_footprint -> robot_i/base_link` を出します。bridge 側の child frame が `robot_i/base_footprint` と一致しない場合、TF ツリーが分断されます。

### `map -> odom` の固定TFは構成次第で競合する

`follower_bt.launch.py` は `map -> odom` をゼロ姿勢の静的TFとして出します。シミュレーションで map と odom を同一座標として扱うなら問題ありませんが、AMCL や SLAM など別ノードが `map -> odom` を出す構成では競合します。

### `PoseStamped` の frame_id が空になり得る

`src/inrof_bt/src/bt/leader_pos_bt.cpp` は `LeaderPos` goal の `start_pos` と `goal_pos` に `header.frame_id` を設定していません。そのため `src/inrof_control/src/path_generate_server.cpp` が生成する `nav_msgs/Path` や、`leader_pos_server.cpp` が publish する `leader/odometry` の frame が空になる可能性があります。TF を使う下流ノードを追加するなら、`map` または `odom` を明示する必要があります。

## tf2 を使った方がよいノード

優先度が高い順に整理します。

| ノード/ファイル | 理由 |
| --- | --- |
| `src/inrof_swerm/src/ros/follower_node.cpp` | 壁回避用の距離場は map 由来ですが、ロボットとリーダーの位置は `Odometry` をそのまま使っています。`map`, `odom`, `robot_i/base_footprint` の関係を tf2 で引き、制御計算の座標系を `map` または `odom` に統一するのが安全です。 |
| `src/inrof_control/src/path_generate_server.cpp` | `start_pos.header.frame_id` を `Path.header.frame_id` にコピーしていますが、`goal_pos` や `waypoints` が同じ frame か確認していません。tf2 で `planning_frame`、例えば `map` に変換してから経路生成するべきです。最低でも frame が空または不一致なら reject する処理が必要です。 |
| `src/inrof_swerm/src/ros/boid_node.cpp` | 複数ロボットの `Odometry` を同じ座標系として混ぜています。シミュレーションでは成立しても、実機や namespace 付きTFに広げるなら、各 `robot_i/base_footprint` を共通フレームに変換する方が堅牢です。 |
| `src/inrof_swerm/src/ros/sheep_node.cpp` | `boid_node.cpp` と同様に、複数ロボットと `leader/odometry` を同一フレーム前提で扱っています。 |
| `src/inrof_control/src/leader_pos_server.cpp` | 仮想リーダー位置を `leader/odometry` として publish していますが、TF上の `leader` frame は出していません。他ノードがTFでリーダーを参照するなら、`map` または `odom` から `leader/base_footprint` への `TransformBroadcaster` を追加すると扱いやすくなります。 |
| `src/inrof_control/src/inrof_control/leader_rl.py` | RL 観測で `robot_i/odometry` を直接使っています。学習時と実行時で frame がずれると壊れやすいので、policy に渡す前に tf2 で固定フレームへそろえる設計が望ましいです。 |

まず手を付けるなら `follower_node.cpp` と `path_generate_server.cpp` が本命です。前者は map 由来の距離場と odometry 由来の位置を混ぜるリスクがあり、後者は frame 不一致を検出できないためです。

## tf2 より先に直した方がよい箇所

- `src/inrof_bt/src/bt/leader_pos_bt.cpp`: `PoseStamped.header.frame_id` を設定する。BT の入力ポートに `frame_id` を追加するか、当面は `map` などの固定値を入れる。
- `src/inrof_sim/urdf/robot.urdf`: `OdometryPublisher` plugin に `robot_base_frame` を明示する。

## 実行時の確認コマンド

TF ツリーが期待通りつながっているかは、launch 実行後に次のコマンドで確認します。

```bash
ros2 run tf2_ros tf2_echo odom robot_0/base_footprint
ros2 run tf2_ros tf2_echo map robot_0/base_footprint
ros2 run tf2_tools view_frames
```

Odometry の frame も同時に確認します。

```bash
ros2 topic echo /robot_0/odometry --once
ros2 topic echo /tf --once
```

`/robot_0/odometry.header.frame_id`, `/robot_0/odometry.child_frame_id`, `/tf` の `header.frame_id` と `child_frame_id` が、`robot_state_publisher` の frame と一致しているかを見るのが重要です。
