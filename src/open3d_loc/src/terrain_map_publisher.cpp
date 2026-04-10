#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class TerrainMapPublisher : public rclcpp::Node
{
public:
    TerrainMapPublisher() : Node("terrain_map_publisher")
    {
        this->declare_parameter<std::string>("input_topic", "/cloud_registered_body_1");
        this->declare_parameter<std::string>("output_topic", "/terrain_map");
        this->declare_parameter<double>("ground_quantile", 0.05);

        std::string input_topic;
        std::string output_topic;
        double ground_quantile = 0.05;

        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("ground_quantile", ground_quantile);

        ground_quantile_ = std::max(0.0, std::min(1.0, ground_quantile));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic, rclcpp::SensorDataQoS());
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&TerrainMapPublisher::PointCloudCallback, this, std::placeholders::_1));
    }

private:
    struct PointXYZ
    {
        float x;
        float y;
        float z;
    };

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<PointXYZ> points;
        std::vector<float> z_values;
        points.reserve(msg->width * msg->height);
        z_values.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            points.push_back({x, y, z});
            z_values.push_back(z);
        }

        if (points.empty())
        {
            return;
        }

        std::sort(z_values.begin(), z_values.end());
        const auto index = static_cast<size_t>((z_values.size() - 1) * ground_quantile_);
        const float ground_z = z_values[index];

        sensor_msgs::msg::PointCloud2 terrain_msg;
        sensor_msgs::PointCloud2Modifier modifier(terrain_msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(points.size());

        terrain_msg.header = msg->header;
        terrain_msg.height = 1;
        terrain_msg.width = static_cast<uint32_t>(points.size());
        terrain_msg.is_dense = false;

        sensor_msgs::PointCloud2Iterator<float> out_x(terrain_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(terrain_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(terrain_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> out_intensity(terrain_msg, "intensity");

        for (const auto &point : points)
        {
            *out_x = point.x;
            *out_y = point.y;
            *out_z = point.z;
            *out_intensity = point.z - ground_z;

            ++out_x;
            ++out_y;
            ++out_z;
            ++out_intensity;
        }

        publisher_->publish(terrain_msg);
    }

    double ground_quantile_ = 0.05;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
