#include <airsim_ros_wrapper_px4_images.h>
#include "common/AirSimSettings.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace std::placeholders;

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

AirsimROSWrapper::AirsimROSWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_img, const std::shared_ptr<rclcpp::Node> nh_lidar, const std::string& host_ip)
    : is_used_lidar_timer_cb_queue_(false)
    , is_used_img_timer_cb_queue_(false)
    , airsim_settings_parser_(host_ip)
    , host_ip_(host_ip)
    , airsim_client_images_(host_ip)
    , nh_(nh)
    , nh_img_(nh_img)
    , isENU_(false)
{

    if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeBoth) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to DRONE mode");
    }
    /*else if{
        airsim_mode_ = AIRSIM_MODE::CAR;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to CAR mode");
    }*/
    else{
        airsim_mode_ = AIRSIM_MODE::BOTH;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to airground mode");
    }
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

    initialize_ros();

    RCLCPP_INFO(nh_->get_logger(), "AirsimROSWrapper Initialized!");
}

void AirsimROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try {
        airsim_client_images_.confirmConnection();
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, something went wrong.\n%s", msg.c_str());
        rclcpp::shutdown();
    }
}

void AirsimROSWrapper::initialize_ros()
{

    // ros params
    nh_->get_parameter("is_vulkan", is_vulkan_);
    isENU_ = 1; 
    // todo enforce dynamics constraints in this node as well?
    // nh_->get_parameter("max_vert_vel_", max_vert_vel_);
    // nh_->get_parameter("max_horz_vel", max_horz_vel_)

    create_ros_pubs_from_settings_json();
    //airsim_control_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //airsim_control_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_control_every_n_sec), std::bind(&AirsimROSWrapper::drone_state_timer_cb, this), airsim_control_callback_group_);
}

void AirsimROSWrapper::create_ros_pubs_from_settings_json()
{
    // subscribe to control commands on global nodehandle
    
    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    size_t lidar_cnt = 0;

    image_transport::ImageTransport image_transporter(nh_);

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;
        const std::string topic_prefix = "~/" + curr_vehicle_name;

        std::cout<<"current vehicle name "<<vehicle_setting->vehicle_type<<std::endl;
        if(vehicle_setting->vehicle_type == "px4multirotor"){
            for (auto& curr_camera_elem : vehicle_setting->cameras) {
                auto& camera_setting = curr_camera_elem.second;
                auto& curr_camera_name = curr_camera_elem.first;

                set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
                //append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
                // camera_setting.gimbal
                std::vector<ImageRequest> current_image_request_vec;
                current_image_request_vec.clear();

                // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
                for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                    auto& capture_setting = curr_capture_elem.second;

                    // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
                    // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
                    if (!(std::isnan(capture_setting.fov_degrees))) {
                        ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                        // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                        if (curr_image_type == ImageType::Scene || curr_image_type == ImageType::Segmentation || curr_image_type == ImageType::SurfaceNormals || curr_image_type == ImageType::Infrared) {
                            current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                        }
                        // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                        else {
                            current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                        }

                        const std::string camera_topic = topic_prefix + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type);
                        image_pub_vec_.push_back(image_transporter.advertise(camera_topic, 1));
                        cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic + "/camera_info", 10));
                        camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                    }
                }
                // push back pair (vector of image captures, current vehicle name)
                airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));
            }
        }

    } 
    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        nh_->get_parameter("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        auto cb = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_img_callback_groups_.push_back(cb);
        airsim_img_response_timer_ = nh_img_->create_wall_timer(std::chrono::duration<double>(update_airsim_img_response_every_n_sec), std::bind(&AirsimROSWrapper::img_response_timer_cb, this), cb);
        is_used_img_timer_cb_queue_ = true;
    }

    // lidars update on their own callback/thread at a given rate

    initialize_airsim();
}

// QoS - The depth of the publisher message queue.
// more details here - https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html


tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}




// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::convert_tf_msg_to_enu(geometry_msgs::msg::TransformStamped& tf_msg)
{
    std::swap(tf_msg.transform.translation.x, tf_msg.transform.translation.y);
    std::swap(tf_msg.transform.rotation.x, tf_msg.transform.rotation.y);
    tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
    tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const
{
    geometry_msgs::msg::Transform optical_tf = body_tf; //same translation
    auto opticalQ = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
    if (1 or isENU_)
        opticalQ *= msr::airlib::Quaternionr(0.7071068, -0.7071068, 0, 0); //CamOptical in CamBodyENU is rmat[1,0,0;0,0,-1;0,1,0]==xyzw[-0.7071068,0,0,0.7071068]
    else
        opticalQ *= msr::airlib::Quaternionr(0.5, 0.5, 0.5, 0.5); //CamOptical in CamBodyNED is rmat[0,0,1;1,0,0;0,1,0]==xyzw[0.5,0.5,0.5,0.5]
    optical_tf.rotation.w = opticalQ.w();
    optical_tf.rotation.x = opticalQ.x();
    optical_tf.rotation.y = opticalQ.y();
    optical_tf.rotation.z = opticalQ.z();
    return optical_tf;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll, rotation.pitch, rotation.yaw);
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();
    transform.rotation.w = quat.w();

    return transform;
}
geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();

    return transform;
}
void AirsimROSWrapper::img_response_timer_cb()
{
    try {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_) {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);
            RCLCPP_ERROR(nh_->get_logger(), "getting image from %s\n", airsim_img_request_vehicle_name_pair.second.c_str());

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) {
                process_and_publish_img_response(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get image response.\n%s", msg.c_str());
    }
}


std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                                     const rclcpp::Time curr_ros_time,
                                                                                     const std::string frame_id)
{
    unused(curr_ros_time);
    std::shared_ptr<sensor_msgs::msg::Image> img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.image_data_uint8.size() / img_response.height;
    img_msg_ptr->header.stamp = rclcpp::Time(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    //img_msg_ptr->header.frame_id = "warty/multisense_front_optical_frame";
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                                           const rclcpp::Time curr_ros_time,
                                                                                           const std::string frame_id)
{
    unused(curr_ros_time);
    auto depth_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    depth_img_msg->width = img_response.width;
    depth_img_msg->height = img_response.height;
    depth_img_msg->data.resize(img_response.image_data_float.size() * sizeof(float));
    memcpy(depth_img_msg->data.data(), img_response.image_data_float.data(), depth_img_msg->data.size());
    depth_img_msg->encoding = "32FC1";
    depth_img_msg->step = depth_img_msg->data.size() / img_response.height;
    depth_img_msg->is_bigendian = 0;
    depth_img_msg->header.stamp = rclcpp::Time(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    //depth_img_msg->header.frame_id = "warty/multisense_front_optical_frame";
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::msg::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& camera_name,
                                                                 const CameraSetting& camera_setting,
                                                                 const CaptureSetting& capture_setting) const
{
    unused(camera_setting);
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    //cam_info_msg.header.frame_id = "warty/multisense_front_optical_frame";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    rclcpp::Time curr_ros_time = nh_->now();
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec) {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope.
        // Ideally, we should loop over cameras and then captures, and publish only one tf.
        publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs

        camera_info_msg_vec_[img_response_idx_internal].header.stamp = rclcpp::Time(curr_img_response.time_stamp);
        cam_info_pub_vec_[img_response_idx_internal]->publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(get_depth_img_msg_from_response(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(get_img_msg_from_response(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame.
// We first do a change of basis to camera optical frame (Z forward, X right, Y down)
void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    unused(ros_time);
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = frame_id + "/" + child_frame_id + "_body";
    cam_tf_body_msg.transform = get_transform_msg_from_airsim(img_response.camera_position, img_response.camera_orientation);

    if (1 or isENU_) {
        convert_tf_msg_to_enu(cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = frame_id + "/" + child_frame_id + "_optical";
    cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(cam_tf_body_msg.transform);

    tf_broadcaster_->sendTransform(cam_tf_body_msg);
    tf_broadcaster_->sendTransform(cam_tf_optical_msg);
}

