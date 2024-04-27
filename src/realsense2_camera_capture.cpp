#include "realsense2_camera_capture/realsense2_camera_capture.hpp"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace realsense2_camera_capture
{

static const rmw_qos_profile_t rmw_qos_profile_latched = {RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                          1,
                                                          RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                          RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                                          RMW_QOS_DEADLINE_DEFAULT,
                                                          RMW_QOS_LIFESPAN_DEFAULT,
                                                          RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                          RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                          false};

static void create_directories(const std::string& dir_name)
{
    if (!std::filesystem::exists(dir_name)) std::filesystem::create_directories(dir_name);
}

static void save_camera_info(const std::string& output, const sensor_msgs::msg::CameraInfo& info)
{
    YAML::Node yaml;
    yaml["height"] = info.height;
    yaml["width"] = info.width;
    yaml["p"] = std::vector<float>(info.p.begin(), info.p.end());
    yaml["d"] = std::vector<float>(info.d.begin(), info.d.end());
    yaml["p"] = std::vector<float>(info.p.begin(), info.p.end());
    yaml["r"] = std::vector<float>(info.r.begin(), info.r.end());
    yaml["distortion_model"] = info.distortion_model;
    yaml["binning_x"] = info.binning_x;
    yaml["binning_y"] = info.binning_y;
    yaml["roi.height"] = info.roi.height;
    yaml["roi.width"] = info.roi.width;
    yaml["roi.x_offset"] = info.roi.x_offset;
    yaml["roi.y_offset"] = info.roi.y_offset;
    yaml["roi.do_rectify"] = info.roi.do_rectify;
    YAML::Emitter out;
    out << yaml;
    std::ofstream file(output);
    file << out.c_str();
    file.close();
}

static void save_extrinsics(const std::string& output,
                            const realsense2_camera_msgs::msg::Extrinsics& extr)
{
    YAML::Node yaml;
    yaml["rotation"] = std::vector<float>(extr.rotation.begin(), extr.rotation.end());
    yaml["translation"] = std::vector<float>(extr.translation.begin(), extr.translation.end());
    YAML::Emitter out;
    out << yaml;
    std::ofstream file(output);
    file << out.c_str();
    file.close();
}

RealSense2CameraCaptureNode::RealSense2CameraCaptureNode(const rclcpp::NodeOptions& options)
    : Node("realsense2_camera_capture", options)
{
    // Declare Parameters
    this->output_dir_ = this->declare_parameter<std::string>("output_dir", "output");
    const std::string sub_rgb_camera_info_topic_name = this->declare_parameter<std::string>(
        "sub_rgb_camera_info_topic_name", "/camera/camera/color/camera_info");
    const std::string sub_rgb_image_topic_name = this->declare_parameter<std::string>(
        "sub_rgb_image_topic_name", "/camera/camera/color/image_raw");

    const std::string sub_depth_camera_info_topic_name = this->declare_parameter<std::string>(
        "sub_depth_camera_info_topic_name", "/camera/camera/depth/camera_info");
    const std::string sub_depth_image_topic_name = this->declare_parameter<std::string>(
        "sub_depth_image_topic_name", "/camera/camera/depth/image_rect_raw");

    const std::string sub_extr_topic_name = this->declare_parameter<std::string>(
        "sub_extr_topic_name", "/camera/camera/extrinsics/depth_to_color");

    const bool use_exact_sync = this->declare_parameter<bool>("use_exact_sync", true);
    const size_t queue_size = this->declare_parameter<int32_t>("queue_size", 10);

    // Create Directories
    create_directories(this->output_dir_);

    // Create TimeSync Subscription
    if (use_exact_sync)
    {
        this->exact_sync_ = std::make_shared<ExactSynchronizer>(
            ExactSyncPolicy(queue_size), sub_rgb_camera_info_, sub_rgb_image_,
            sub_depth_camera_info_, sub_depth_image_);
        this->exact_sync_->registerCallback(
            std::bind(&RealSense2CameraCaptureNode::rgbd_callback_, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }
    else
    {
        this->sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_rgb_camera_info_,
                                                     sub_rgb_image_, sub_depth_camera_info_,
                                                     sub_depth_image_);
        this->sync_->registerCallback(std::bind(&RealSense2CameraCaptureNode::rgbd_callback_, this,
                                                std::placeholders::_1, std::placeholders::_2,
                                                std::placeholders::_3, std::placeholders::_4));
    }

    this->sub_rgb_camera_info_.subscribe(this, sub_rgb_camera_info_topic_name);
    this->sub_rgb_image_.subscribe(this, sub_rgb_image_topic_name);
    this->sub_depth_camera_info_.subscribe(this, sub_depth_camera_info_topic_name);
    this->sub_depth_image_.subscribe(this, sub_depth_image_topic_name);

    // Create Subscription
    this->sub_extr_ = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        sub_extr_topic_name,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_latched),
                    rmw_qos_profile_latched),
        std::bind(&RealSense2CameraCaptureNode::extr_callback_, this, std::placeholders::_1));
}

void RealSense2CameraCaptureNode::rgbd_callback_(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr rgb_camera_info,
    const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_camera_info,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
{
    static bool is_first = true;
    const long sec = rgb_msg->header.stamp.sec;
    const uint32_t nanosec = rgb_msg->header.stamp.nanosec;
    std::string nanosec_str = std::to_string(nanosec);
    nanosec_str.insert(0, 9 - nanosec_str.length(), '0');
    const std::string time_str = std::to_string(sec) + "." + nanosec_str;

    if (is_first)
    {
        const std::string rgb_output = output_dir_ + "/rgb_camera_info.yaml";
        save_camera_info(rgb_output, *rgb_camera_info);
        RCLCPP_INFO(this->get_logger(), "RGB Camera Info save as %s", rgb_output.c_str());

        const std::string depth_output = output_dir_ + "/depth_camera_info.yaml";
        save_camera_info(depth_output, *depth_camera_info);
        RCLCPP_INFO(this->get_logger(), "Depth Camera Info save as %s", depth_output.c_str());
        is_first = false;
    }
    const std::string rgb_dir = this->output_dir_ + "/rgb";
    const std::string depth_dir = this->output_dir_ + "/depth";
    create_directories(rgb_dir);
    create_directories(depth_dir);

    const std::string rgb_path = rgb_dir + "/" + time_str + ".jpg";
    const auto rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
    cv::imwrite(rgb_path, rgb_img);

    const std::string depth_path = depth_dir + "/" + time_str + ".png";
    const auto depth_img = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
    cv::imwrite(depth_path, depth_img);
}

void RealSense2CameraCaptureNode::extr_callback_(
    const realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr msg)
{
    const std::string output = output_dir_ + "/extrinsics.yaml";
    save_extrinsics(output, *msg);
    RCLCPP_INFO(this->get_logger(), "Extrinsics save as %s", output.c_str());
}

} // namespace realsense2_camera_capture

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense2_camera_capture::RealSense2CameraCaptureNode)
