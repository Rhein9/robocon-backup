#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>

class PcdMapPublisher : public rclcpp::Node
{
public:
    PcdMapPublisher() : Node("pcd_map_publisher")
    {
        this->declare_parameter<std::string>("path_map", "");
        this->declare_parameter<std::string>("map_topic", "/nav2_map");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<double>("padding", 1.0);
        this->declare_parameter<bool>("auto_ground_height", true);
        this->declare_parameter<double>("ground_quantile", 0.05);
        this->declare_parameter<double>("ground_z", 0.0);
        this->declare_parameter<double>("min_obstacle_height_from_ground", 0.15);
        this->declare_parameter<double>("max_obstacle_height_from_ground", 1.8);
        this->declare_parameter<double>("obstacle_dilation_radius", 0.15);
        this->declare_parameter<double>("publish_period_s", 5.0);
        this->declare_parameter<int>("occupied_value", 100);
        this->declare_parameter<int>("free_value", 0);
        this->declare_parameter<std::string>("output_pgm_path", "");
        this->declare_parameter<std::string>("output_yaml_path", "");

        std::string map_path;
        std::string map_topic;
        std::string map_frame;
        double resolution = 0.1;
        double padding = 1.0;
        double ground_quantile = 0.05;
        double ground_z = 0.0;
        double min_obstacle_height = 0.15;
        double max_obstacle_height = 1.8;
        double obstacle_dilation_radius = 0.15;
        double publish_period_s = 5.0;
        int occupied_value = 100;
        int free_value = 0;
        bool auto_ground_height = true;
        std::string output_pgm_path;
        std::string output_yaml_path;

        this->get_parameter("path_map", map_path);
        this->get_parameter("map_topic", map_topic);
        this->get_parameter("map_frame", map_frame);
        this->get_parameter("resolution", resolution);
        this->get_parameter("padding", padding);
        this->get_parameter("auto_ground_height", auto_ground_height);
        this->get_parameter("ground_quantile", ground_quantile);
        this->get_parameter("ground_z", ground_z);
        this->get_parameter("min_obstacle_height_from_ground", min_obstacle_height);
        this->get_parameter("max_obstacle_height_from_ground", max_obstacle_height);
        this->get_parameter("obstacle_dilation_radius", obstacle_dilation_radius);
        this->get_parameter("publish_period_s", publish_period_s);
        this->get_parameter("occupied_value", occupied_value);
        this->get_parameter("free_value", free_value);
        this->get_parameter("output_pgm_path", output_pgm_path);
        this->get_parameter("output_yaml_path", output_yaml_path);

        if (map_path.empty())
        {
            throw std::runtime_error("Parameter 'path_map' is empty, cannot build occupancy map.");
        }
        if (resolution <= 0.0)
        {
            throw std::runtime_error("Parameter 'resolution' must be > 0.");
        }

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local();
        map_qos.reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);

        occupancy_grid_ = BuildOccupancyGrid(
            map_path,
            map_frame,
            resolution,
            padding,
            auto_ground_height,
            ground_quantile,
            ground_z,
            min_obstacle_height,
            max_obstacle_height,
            obstacle_dilation_radius,
            occupied_value,
            free_value);

        if (!output_pgm_path.empty() || !output_yaml_path.empty())
        {
            if (output_pgm_path.empty() || output_yaml_path.empty())
            {
                throw std::runtime_error(
                    "Parameters 'output_pgm_path' and 'output_yaml_path' must be set together.");
            }

            WriteMapFiles(occupancy_grid_, output_pgm_path, output_yaml_path);
        }

        PublishMap();

        if (publish_period_s > 0.0)
        {
            auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(publish_period_s));
            timer_ = this->create_wall_timer(period, std::bind(&PcdMapPublisher::PublishMap, this));
        }
    }

private:
    nav_msgs::msg::OccupancyGrid BuildOccupancyGrid(
        const std::string &map_path,
        const std::string &map_frame,
        double resolution,
        double padding,
        bool auto_ground_height,
        double ground_quantile,
        double ground_z,
        double min_obstacle_height,
        double max_obstacle_height,
        double obstacle_dilation_radius,
        int occupied_value,
        int free_value)
    {
        auto pcd_map = std::make_shared<open3d::geometry::PointCloud>();
        if (!open3d::io::ReadPointCloud(map_path, *pcd_map))
        {
            throw std::runtime_error("Failed to load point cloud map from: " + map_path);
        }
        if (pcd_map->points_.empty())
        {
            throw std::runtime_error("Point cloud map is empty: " + map_path);
        }

        std::vector<double> z_values;
        z_values.reserve(pcd_map->points_.size());

        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto &point : pcd_map->points_)
        {
            min_x = std::min(min_x, point.x());
            min_y = std::min(min_y, point.y());
            max_x = std::max(max_x, point.x());
            max_y = std::max(max_y, point.y());
            z_values.push_back(point.z());
        }

        if (auto_ground_height)
        {
            std::sort(z_values.begin(), z_values.end());
            if (ground_quantile < 0.0)
            {
                ground_quantile = 0.0;
            }
            if (ground_quantile > 1.0)
            {
                ground_quantile = 1.0;
            }
            const auto idx = static_cast<size_t>((z_values.size() - 1) * ground_quantile);
            ground_z = z_values[idx];
        }

        const double origin_x = min_x - padding;
        const double origin_y = min_y - padding;
        const double size_x = (max_x - min_x) + (2.0 * padding);
        const double size_y = (max_y - min_y) + (2.0 * padding);

        const auto width = static_cast<uint32_t>(std::ceil(size_x / resolution));
        const auto height = static_cast<uint32_t>(std::ceil(size_y / resolution));
        if (width == 0U || height == 0U)
        {
            throw std::runtime_error("Generated occupancy map has invalid size.");
        }

        std::vector<int8_t> data(width * height, static_cast<int8_t>(free_value));
        std::vector<std::pair<int, int>> occupied_cells;

        const double min_z = ground_z + min_obstacle_height;
        const double max_z = ground_z + max_obstacle_height;

        for (const auto &point : pcd_map->points_)
        {
            if (point.z() < min_z || point.z() > max_z)
            {
                continue;
            }

            const int gx = static_cast<int>(std::floor((point.x() - origin_x) / resolution));
            const int gy = static_cast<int>(std::floor((point.y() - origin_y) / resolution));
            if (gx < 0 || gy < 0 || gx >= static_cast<int>(width) || gy >= static_cast<int>(height))
            {
                continue;
            }

            data[static_cast<size_t>(gy) * width + static_cast<size_t>(gx)] = static_cast<int8_t>(occupied_value);
            occupied_cells.emplace_back(gx, gy);
        }

        const int dilation_cells = static_cast<int>(std::ceil(obstacle_dilation_radius / resolution));
        if (dilation_cells > 0 && !occupied_cells.empty())
        {
            auto dilated_data = data;
            const int squared_radius = dilation_cells * dilation_cells;
            for (const auto &cell : occupied_cells)
            {
                for (int dy = -dilation_cells; dy <= dilation_cells; ++dy)
                {
                    for (int dx = -dilation_cells; dx <= dilation_cells; ++dx)
                    {
                        if ((dx * dx + dy * dy) > squared_radius)
                        {
                            continue;
                        }

                        const int nx = cell.first + dx;
                        const int ny = cell.second + dy;
                        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height))
                        {
                            continue;
                        }

                        dilated_data[static_cast<size_t>(ny) * width + static_cast<size_t>(nx)] = static_cast<int8_t>(occupied_value);
                    }
                }
            }
            data = std::move(dilated_data);
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = map_frame;
        grid.info.map_load_time = this->now();
        grid.info.resolution = static_cast<float>(resolution);
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data = std::move(data);

        size_t occupied_count = 0;
        for (const auto value : grid.data)
        {
            if (value >= static_cast<int8_t>(occupied_value))
            {
                ++occupied_count;
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded PCD map '%s' (%zu pts), estimated ground_z=%.3f, occupancy size=%ux%u @ %.3fm, occupied=%zu cells",
            map_path.c_str(),
            pcd_map->points_.size(),
            ground_z,
            width,
            height,
            resolution,
            occupied_count);

        return grid;
    }

    void WriteMapFiles(
        const nav_msgs::msg::OccupancyGrid &grid,
        const std::string &pgm_path,
        const std::string &yaml_path)
    {
        std::ofstream pgm_stream(pgm_path, std::ios::binary);
        if (!pgm_stream)
        {
            throw std::runtime_error("Failed to open PGM output path: " + pgm_path);
        }

        pgm_stream << "P5\n";
        pgm_stream << "# CREATOR: open3d_loc pcd_map_publisher\n";
        pgm_stream << grid.info.width << " " << grid.info.height << "\n255\n";

        for (uint32_t y = 0; y < grid.info.height; ++y)
        {
            for (uint32_t x = 0; x < grid.info.width; ++x)
            {
                const auto index =
                    static_cast<size_t>(x) +
                    static_cast<size_t>(grid.info.height - y - 1) * grid.info.width;
                const auto value = grid.data[index];

                unsigned char pixel = 205;
                if (value < 0)
                {
                    pixel = 205;
                }
                else if (value >= 65)
                {
                    pixel = 0;
                }
                else
                {
                    pixel = 254;
                }

                pgm_stream.write(reinterpret_cast<const char *>(&pixel), sizeof(pixel));
            }
        }

        std::ofstream yaml_stream(yaml_path);
        if (!yaml_stream)
        {
            throw std::runtime_error("Failed to open YAML output path: " + yaml_path);
        }

        yaml_stream << "image: " << pgm_path << "\n";
        yaml_stream << "mode: trinary\n";
        yaml_stream << "resolution: " << grid.info.resolution << "\n";
        yaml_stream << "origin: ["
                    << grid.info.origin.position.x << ", "
                    << grid.info.origin.position.y << ", 0.0]\n";
        yaml_stream << "negate: 0\n";
        yaml_stream << "occupied_thresh: 0.65\n";
        yaml_stream << "free_thresh: 0.25\n";

        RCLCPP_INFO(
            this->get_logger(),
            "Exported occupancy map files: '%s' and '%s'",
            pgm_path.c_str(),
            yaml_path.c_str());
    }

    void PublishMap()
    {
        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_.info.map_load_time = occupancy_grid_.header.stamp;
        map_pub_->publish(occupancy_grid_);
    }

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
