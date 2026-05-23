# Wall Repulsion With Distance Field

This note summarizes the map-data design discussed for adding wall repulsion to
`BoidCore`.

## Goal

The goal is to make each boid avoid walls by adding a repulsive force from the
nearest wall.

For this purpose, the raw occupancy grid is useful for storage and communication,
but it is not the most convenient form for force computation. The force needs:

- distance to the nearest wall
- direction away from the wall

Those are easier to compute from a distance field.

## Recommended Data Flow

Use ROS/Nav2 standard data at the ROS boundary, and use a lightweight internal
map in the core layer.

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
boid_node.cpp converts ROS message
        |
        v
GridMap / DistanceFieldMap
        |
        v
boid_core.cpp computes wall force
```

`boid_core.cpp` should avoid depending directly on ROS message types. It is a
calculation core, so it is better to pass plain C++ data structures into it.

## GridMap

`GridMap` is a lightweight form of `nav_msgs/msg/OccupancyGrid`.

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

The occupancy value is normally interpreted as:

```text
-1   unknown
0    free
100  occupied
```

In this project, a cell can be treated as a wall when:

```cpp
map.data[index] >= 65
```

For real robots, unknown cells may also be treated as walls for safety.

## Why Not MatrixXd?

The occupancy grid itself does not need to be converted to `Eigen::MatrixXd`.
The occupancy values are integer states, not floating-point continuous values.

Good choices are:

- `std::vector<int8_t>` for occupancy
- `Eigen::Map` if matrix-style indexing is convenient
- `std::vector<float>` or `Eigen::ArrayXXf` for distance fields

If using Eigen to view occupancy data, use row-major layout because ROS
`OccupancyGrid` stores data as:

```cpp
index = y * width + x;
```

Example:

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

## Distance Field

A distance field is a map where each cell stores the distance to the nearest wall.

Example:

```text
# = wall
. = free

# . . . .
# . . . .
# . . . .
```

With `resolution = 0.05`, the distance field is approximately:

```text
0.00 0.05 0.10 0.15 0.20
0.00 0.05 0.10 0.15 0.20
0.00 0.05 0.10 0.15 0.20
```

The wall repulsion rule then becomes simple:

```text
near wall -> strong force
far wall  -> weak or zero force
```

Recommended structure:

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

## Building A Distance Field

The simple method discussed here is multi-source Dijkstra on an 8-neighbor grid.

It works like this:

1. Initialize every cell distance to infinity.
2. Set every wall cell distance to `0`.
3. Push all wall cells into a priority queue.
4. Pop the cell with the smallest distance.
5. Propagate its distance to the 8 neighboring cells.
6. If a shorter distance is found, update the neighbor and push it into the queue.
7. Continue until the queue is empty.

This computes an approximate distance to the nearest wall on the 8-neighbor grid.

The movement cost is:

```text
horizontal / vertical: resolution
diagonal:              resolution * sqrt(2)
```

Example implementation:

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

This is not an exact Euclidean distance transform, but it is usually good enough
for wall repulsion in a boid controller.

## Gradient

The gradient of the distance field points toward increasing distance. Since
distance increases as we move away from a wall, the gradient gives the direction
away from the wall.

For an inner cell:

```cpp
grad_x = (D[y][x + 1] - D[y][x - 1]) / (2.0 * resolution);
grad_y = (D[y + 1][x] - D[y - 1][x]) / (2.0 * resolution);
```

In vector form:

```cpp
Eigen::Vector2d grad(grad_x, grad_y);
Eigen::Vector2d away_from_wall = grad.normalized();
```

Boundary cells need special handling because `x - 1`, `x + 1`, `y - 1`, or
`y + 1` may be outside the map. Options are:

- skip gradient computation at boundaries
- use one-sided differences
- clamp neighbor indices to the map boundary

## Wall Repulsion Force

After computing distance and gradient, wall force can be added to the normal boid
force.

Example:

```cpp
Eigen::Vector2d wall_force = Eigen::Vector2d::Zero();

if (d < wall_range && grad.norm() > 1e-6) {
    const double safe_d = std::max<double>(d, 0.001);
    Eigen::Vector2d dir = grad.normalized();
    double strength = k_wall * (1.0 / safe_d - 1.0 / wall_range);
    wall_force = strength * dir;
}
```

Then combine it with the existing boid force:

```cpp
cmd_vel_i =
    base_boid_force +
    wall_force;
```

The final velocity should still be limited by `max_vel`.

## Suggested Placement

Suggested responsibilities:

- `boid_node.cpp`
  - subscribe to `/map`
  - convert `nav_msgs/msg/OccupancyGrid` to `GridMap`
  - build or update `DistanceFieldMap`
  - pass the internal map to `BoidCore`

- `boid_core.cpp`
  - stay independent from ROS
  - use `DistanceFieldMap`
  - compute wall repulsion force
  - combine it with separation, alignment, and gravity

This keeps ROS message handling in the ROS layer and keeps the boid algorithm
easy to test as plain C++.
