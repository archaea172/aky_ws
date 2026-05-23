# 距離場を使った壁反発

このメモは、`BoidCore` に壁への反発力を追加するための地図データ設計をまとめたものです。

## 目的

目的は、各 boid が壁を避けるように、最も近い壁からの反発力を加えることです。

この用途では、生の占有グリッドは保存や通信には便利ですが、力の計算には少し扱いにくいです。
反発力の計算に必要なのは、主に次の2つです。

- 最も近い壁までの距離
- 壁から離れる方向

これらは距離場から計算するのが扱いやすいです。

## 推奨するデータの流れ

ROSとの境界ではROS/Nav2の標準データを使い、計算コアでは軽量な内部表現を使います。

```text
map yaml + pgm
        |
        v
nav2_map_server
        |
        v
/map: nav_msgs/msg/OccupancyGrid
        |
        v
boid_node.cpp でROSメッセージを変換
        |
        v
GridMap / DistanceFieldMap
        |
        v
boid_core.cpp で壁反発力を計算
```

`boid_core.cpp` はROSメッセージ型に直接依存しないほうがよいです。
ここは計算コアなので、通常のC++構造体を渡すほうがテストしやすく、責務も分かりやすくなります。

## GridMap

`GridMap` は `nav_msgs/msg/OccupancyGrid` を軽量にした内部表現です。

```cpp
struct GridMap
{
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;
    std::vector<int8_t> data;  // -1 unknown, 0 free, 100 occupied
};
```

占有値は通常、次のように解釈します。

```text
-1   unknown
0    free
100  occupied
```

このプロジェクトでは、例えば次の条件で壁セルとして扱えます。

```cpp
map.data[index] >= 65
```

実機で安全側に倒したい場合は、unknown セルも壁扱いにする選択肢があります。

## MatrixXd にしない理由

占有グリッドそのものを `Eigen::MatrixXd` に変換する必要はありません。
占有値は連続的な浮動小数点値ではなく、整数の状態値だからです。

よい選択肢は次の通りです。

- 占有値は `std::vector<int8_t>` で持つ
- 行列風にアクセスしたい場合は `Eigen::Map` を使う
- 距離場は `std::vector<float>` または `Eigen::ArrayXXf` で持つ

Eigenで占有データを見る場合、ROSの `OccupancyGrid` は次の並びで保存されているため、row-major を使うのが重要です。

```cpp
index = y * width + x;
```

例:

```cpp
using OccupancyMatrix =
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Eigen::Map<const OccupancyMatrix> grid(
    map.data.data(),
    map.height,
    map.width
);

int8_t value = grid(my, mx);
```

## 距離場

距離場とは、各セルに「最も近い壁までの距離」が入っている地図です。

例:

```text
# = wall
. = free

# . . . .
# . . . .
# . . . .
```

`resolution = 0.05` の場合、距離場はおおよそ次のようになります。

```text
0.00 0.05 0.10 0.15 0.20
0.00 0.05 0.10 0.15 0.20
0.00 0.05 0.10 0.15 0.20
```

こうすると、壁反発のルールは単純になります。

```text
壁に近い -> 強い力
壁から遠い -> 弱い力、または力なし
```

推奨する構造体は次の形です。

```cpp
struct DistanceFieldMap
{
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;

    std::vector<int8_t> occupancy;
    std::vector<float> distance;
    std::vector<float> grad_x;
    std::vector<float> grad_y;
};
```

## 距離場の作り方

ここで扱うシンプルな方法は、8近傍グリッド上での multi-source Dijkstra です。

流れは次の通りです。

1. 全セルの距離を infinity で初期化する。
2. 全ての壁セルの距離を `0` にする。
3. 全ての壁セルを priority queue に入れる。
4. 距離が最も小さいセルを取り出す。
5. そのセルの距離を8近傍のセルへ伝播する。
6. より短い距離が見つかったら、その近傍セルを更新して queue に入れる。
7. queue が空になるまで続ける。

これにより、8近傍グリッド上での「最も近い壁までの近似距離」を計算できます。

移動コストは次の2種類です。

```text
上下左右: resolution
斜め:     resolution * sqrt(2)
```

実装例:

```cpp
DistanceFieldMap make_distance_field(const GridMap& map)
{
    const int w = map.width;
    const int h = map.height;
    const float inf = std::numeric_limits<float>::infinity();

    DistanceFieldMap field;
    field.resolution = map.resolution;
    field.origin_x = map.origin_x;
    field.origin_y = map.origin_y;
    field.width = w;
    field.height = h;
    field.occupancy = map.data;
    field.distance.assign(w * h, inf);

    auto idx = [w](int x, int y) {
        return y * w + x;
    };

    using Item = std::pair<float, int>;  // distance, index
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = idx(x, y);

            if (map.data[i] >= 65) {
                field.distance[i] = 0.0f;
                queue.push({0.0f, i});
            }
        }
    }

    const std::array<std::pair<int, int>, 8> dirs = {{
        {-1,  0}, {1,  0}, {0, -1}, {0, 1},
        {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
    }};

    while (!queue.empty()) {
        auto [d, i] = queue.top();
        queue.pop();

        if (d > field.distance[i]) {
            continue;
        }

        int x = i % w;
        int y = i / w;

        for (auto [dx, dy] : dirs) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
                continue;
            }

            float step = (dx != 0 && dy != 0)
                ? static_cast<float>(map.resolution * std::sqrt(2.0))
                : static_cast<float>(map.resolution);

            int ni = idx(nx, ny);
            float nd = d + step;

            if (nd < field.distance[ni]) {
                field.distance[ni] = nd;
                queue.push({nd, ni});
            }
        }
    }

    return field;
}
```

これは厳密なユークリッド距離変換ではありませんが、boid制御の壁反発には十分使いやすい近似です。

## 勾配

距離場の勾配は、距離が増える方向を指します。
壁から離れるほど距離は大きくなるので、勾配は壁から離れる方向になります。

内側のセルでは次のように計算できます。

```cpp
grad_x = (D[y][x + 1] - D[y][x - 1]) / (2.0 * resolution);
grad_y = (D[y + 1][x] - D[y - 1][x]) / (2.0 * resolution);
```

ベクトルで表すと次のようになります。

```cpp
Eigen::Vector2d grad(grad_x, grad_y);
Eigen::Vector2d away_from_wall = grad.normalized();
```

境界セルでは、`x - 1`, `x + 1`, `y - 1`, `y + 1` が地図外になる可能性があります。
対応方法は次のようなものがあります。

- 境界では勾配計算をスキップする
- 片側差分を使う
- 近傍インデックスを地図範囲内に clamp する

## 壁反発力

距離と勾配が計算できれば、通常の boid の力に壁反発力を足せます。

例:

```cpp
Eigen::Vector2d wall_force = Eigen::Vector2d::Zero();

if (d < wall_range && grad.norm() > 1e-6) {
    const double safe_d = std::max<double>(d, 0.001);
    Eigen::Vector2d dir = grad.normalized();
    double strength = k_wall * (1.0 / safe_d - 1.0 / wall_range);
    wall_force = strength * dir;
}
```

既存の boid の力とは次のように合成します。

```cpp
cmd_vel_i =
    base_boid_force +
    wall_force;
```

最後に、これまで通り `max_vel` で速度を制限します。

## 推奨する配置

責務分担は次の形がよいです。

- `boid_node.cpp`
  - `/map` を subscribe する
  - `nav_msgs/msg/OccupancyGrid` を `GridMap` に変換する
  - `DistanceFieldMap` を作成または更新する
  - 内部表現の地図を `BoidCore` に渡す

- `boid_core.cpp`
  - ROSに依存しない
  - `DistanceFieldMap` を使う
  - 壁反発力を計算する
  - separation, alignment, gravity と合成する

この形にすると、ROSメッセージ処理はROS層に閉じ込められ、boidアルゴリズムは通常のC++としてテストしやすくなります。
