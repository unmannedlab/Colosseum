#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <random>

class LidarRealisticGroundPublisher : public rclcpp::Node {
public:
    LidarRealisticGroundPublisher() : Node("lidar_realistic_ground_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/warty/lidar_points", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarRealisticGroundPublisher::publishPointCloud, this));
    }

private:
    void publishPointCloud() {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "warty/lidar_link";
        cloud_msg.height = 1;  // Organized as a single row
        cloud_msg.width = 64000;  // Number of points in the point cloud
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 16;
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.is_dense = true;

        // Define point fields (x, y, z, intensity)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

        modifier.resize(cloud_msg.width);

        // Initialize iterators
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

        std::default_random_engine generator;
        std::uniform_real_distribution<float> dist_x(-50.0, 50.0);   // X spread from -50m to 50m
        std::uniform_real_distribution<float> dist_y(-50.0, 50.0);   // Y spread from -50m to 50m
        std::normal_distribution<float> dist_z(0.5, 0.05);  // Z variation centered at 0.5m, small deviation
        std::uniform_real_distribution<float> dist_intensity(0.5, 1.0); // Vary intensity slightly

        // Generate points for **a slightly uneven ground plane** with minor elevation changes
        for (size_t i = 0; i < cloud_msg.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            *iter_x = dist_x(generator);
            *iter_y = dist_y(generator);
            *iter_z = dist_z(generator);  // Adding small terrain variation
            *iter_intensity = dist_intensity(generator);
        }

        publisher_->publish(cloud_msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published realistic ground point cloud to /warty/lidar_points.");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRealisticGroundPublisher>());
    rclcpp::shutdown();
    return 0;
}

