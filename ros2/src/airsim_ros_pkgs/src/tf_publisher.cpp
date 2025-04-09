#include <memory>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class StaticTransformPublisher : public rclcpp::Node {
public:
    StaticTransformPublisher() : Node("static_tf_publisher") {
        // QoS Profile for /tf_static (Latched)
        rclcpp::QoS qos_static(rclcpp::KeepLast(1));
        qos_static.transient_local().reliable();

        // Publisher for /tf_static (latched)
        pub_static_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos_static);
        
        // Publisher for /tf (continuous)
        pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        // Define static transforms
        static_transforms_ = {
            create_transform("warty/base_link", "warty/gps_link", 0.0, 0.0, 0.0, 0, 0, 0, 1),
            create_transform("warty/base_link", "warty/multisense_front_optical_frame", 0.549998, -2.38772e-06, 0.849999, -0.560985, 0.560985, -0.430459, 0.430459),
            create_transform("warty/base_link", "warty/imu_link", 0.0, 0.0, 0.0, 0, 0, 0, 1),
            create_transform("warty/base_link", "warty/lidar_link", -5.406e-06, -3.238e-06, 1.099999, 0, 0, 0, 1),
            create_transform("warty/base_link", "warty/multisense_back_optical_frame", -0.550011, 4.54526e-07, 0.849998, 0.560985, 0.560985, -0.430459, -0.430459),

            create_transform("warty2/base_link", "warty2/gps_link", 0.0, 0.0, 0.0, 0, 0, 0, 1),
            create_transform("warty2/base_link", "warty2/multisense_front_optical_frame", 0.549998, -2.38772e-06, 0.849999, -0.560985, 0.560985, -0.430459, 0.430459),
            create_transform("warty2/base_link", "warty2/imu_link", 0.0, 0.0, 0.0, 0, 0, 0, 1),
            create_transform("warty2/base_link", "warty2/lidar_link", -5.406e-06, -3.238e-06, 1.099999, 0, 0, 0, 1),
            create_transform("warty2/base_link", "warty2/multisense_back_optical_frame", -0.550011, 4.54526e-07, 0.849998, 0.560985, 0.560985, -0.430459, -0.430459)
            //create_transform("world_ned", "world_enu", 0, 0, 0, 1.57, 0, 3.14)
        };

        // Define dynamic transforms
        dynamic_transforms_ = {
            create_transform("warty/base_link", "warty/right_diff_unit_link", -0.0000355, -0.5683192, 0.3001780, -0.0001916, 0.0027995, 0.0000379, 0.999996),
            create_transform("warty/base_link", "warty/left_diff_unit_link", -0.0000378, 0.5685138, 0.3009987, 0.0009023, -0.0059828, 0.0000031, 0.999982),
            create_transform("warty/left_diff_unit_link", "warty/rear_left_wheel_link", -0.4573765, 0.0000018, -0.0129792, 0, 0, 0, 1),
            create_transform("warty/right_diff_unit_link", "warty/front_right_wheel_link", 0.4573825, 0.0000200, -0.0135154, 0, 0, 0, 1),
            create_transform("warty/left_diff_unit_link", "warty/front_left_wheel_link", 0.4573606, 0.0000078, -0.0135917, 0, 0, 0, 1),
            create_transform("warty/right_diff_unit_link", "warty/rear_right_wheel_link", -0.4573574, 0.0000146, -0.0129741, 0, 0, 0, 1),

            create_transform("warty2/base_link", "warty2/right_diff_unit_link", -0.0000355, -0.5683192, 0.3001780, -0.0001916, 0.0027995, 0.0000379, 0.999996),
            create_transform("warty2/base_link", "warty2/left_diff_unit_link", -0.0000378, 0.5685138, 0.3009987, 0.0009023, -0.0059828, 0.0000031, 0.999982),
            create_transform("warty2/left_diff_unit_link", "warty2/rear_left_wheel_link", -0.4573765, 0.0000018, -0.0129792, 0, 0, 0, 1),
            create_transform("warty2/right_diff_unit_link", "warty2/front_right_wheel_link", 0.4573825, 0.0000200, -0.0135154, 0, 0, 0, 1),
            create_transform("warty2/left_diff_unit_link", "warty2/front_left_wheel_link", 0.4573606, 0.0000078, -0.0135917, 0, 0, 0, 1),
            create_transform("warty2/right_diff_unit_link", "warty2/rear_right_wheel_link", -0.4573574, 0.0000146, -0.0129741, 0, 0, 0, 1)
        };

        // Publish static transforms once (latched)
        publish_static_transforms();

        // Timer for publishing dynamic transforms at 15 Hz (every 66.6 ms)
        timer_ = this->create_wall_timer(66ms, std::bind(&StaticTransformPublisher::publish_dynamic_transforms, this));

        RCLCPP_INFO(this->get_logger(), "Continuous TF Publisher started!");
    }

private:
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_static_tf_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
    std::vector<geometry_msgs::msg::TransformStamped> dynamic_transforms_;

    geometry_msgs::msg::TransformStamped create_transform(
        const std::string &parent_frame, const std::string &child_frame,
        double x, double y, double z, double qx, double qy, double qz, double qw) {
        
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        t.transform.rotation.x = qx;
        t.transform.rotation.y = qy;
        t.transform.rotation.z = qz;
        t.transform.rotation.w = qw;
        return t;
    }

    void publish_static_transforms() {
        for (auto &transform : static_transforms_) {
            transform.header.stamp = this->now();
        }

        tf2_msgs::msg::TFMessage msg;
        msg.transforms = static_transforms_;
        pub_static_tf_->publish(msg);

        //RCLCPP_INFO(this->get_logger(), "Published latched static transforms to /tf_static");
    }

    void publish_dynamic_transforms() {
        for (auto &transform : dynamic_transforms_) {
            transform.header.stamp = this->now();
        }

        tf2_msgs::msg::TFMessage msg;
        msg.transforms = dynamic_transforms_;
        pub_tf_->publish(msg);

        //RCLCPP_INFO(this->get_logger(), "Published continuous transforms to /tf");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

