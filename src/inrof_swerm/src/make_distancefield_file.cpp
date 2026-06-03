#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

#include "core/boid_core.hpp"

cv::Mat make_distance_field_image(const DistanceFieldMap& field, float max_dist)
{
    cv::Mat gray(field.height, field.width, CV_8UC1);

    for (int y = 0; y < field.height; ++y) {
        for (int x = 0; x < field.width; ++x) {
            float d = field.distance(y, x);

            if (!std::isfinite(d)) {
                gray.at<uint8_t>(y, x) = 0;
                continue;
            }

            float t = std::clamp(d / max_dist, 0.0f, 1.0f);

            // 壁に近いほど明るい
            gray.at<uint8_t>(y, x) =
                static_cast<uint8_t>((1.0f - t) * 255.0f);
        }
    }

    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_TURBO);

    // OpenCV画像として見るなら上下反転した方が地図座標っぽく見えることが多い
    cv::flip(color, color, 0);

    return color;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("make_distancefield_file");

    nav_msgs::msg::OccupancyGrid rxdata;
    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    bool success = rclcpp::wait_for_message(
        rxdata,
        node,
        "map",
        std::chrono::seconds(4),
        map_qos
    );

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Received occupancy grid, writing to file...");
        GridMap map;
        map.height = rxdata.info.height;
        map.width = rxdata.info.width;
        map.origin_x = rxdata.info.origin.position.x;
        map.origin_y = rxdata.info.origin.position.y;
        const auto &orientation = rxdata.info.origin.orientation;
        map.origin_yaw = std::atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        );
        map.resolution = rxdata.info.resolution;
        map.data = rxdata.data;

        DistanceFieldMap field = convertmap_grid_to_distance(map);

        cv::Mat image = make_distance_field_image(field, 2.0f);
        cv::imshow("distance field", image);
        cv::waitKey(0);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to receive occupancy grid within timeout.");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}