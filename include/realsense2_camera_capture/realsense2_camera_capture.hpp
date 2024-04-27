#pragma once

#include <rclcpp/rclcpp.hpp>

#include "realsense2_camera_msgs/msg/extrinsics.hpp"
// #include "realsense2_camera_msgs/msg/rgbd.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace realsense2_camera_capture
{
class RealSense2CameraCaptureNode : public rclcpp::Node
{
public:
    RealSense2CameraCaptureNode(const rclcpp::NodeOptions& options);

private:
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CameraInfo, // nolint
                                                        sensor_msgs::msg::Image,      // nolint
                                                        sensor_msgs::msg::CameraInfo, // nolint
                                                        sensor_msgs::msg::Image>;     // nolint
    using ExactSyncPolicy =
        message_filters::sync_policies::ExactTime<sensor_msgs::msg::CameraInfo, // nolint
                                                  sensor_msgs::msg::Image,      // nolint
                                                  sensor_msgs::msg::CameraInfo, // nolint
                                                  sensor_msgs::msg::Image>;     // nolint

    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    using ExactSynchronizer = message_filters::Synchronizer<ExactSyncPolicy>;

    void rgbd_callback_(const sensor_msgs::msg::CameraInfo::ConstSharedPtr rgb_camera_info,
                        const sensor_msgs::msg::Image::ConstSharedPtr rgb_image,
                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_camera_info,
                        const sensor_msgs::msg::Image::ConstSharedPtr depth_image);
    void extr_callback_(const realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr msg);

    std::shared_ptr<Synchronizer> sync_;
    std::shared_ptr<ExactSynchronizer> exact_sync_;

    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_rgb_camera_info_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_image_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_depth_camera_info_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_image_;

    rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_extr_;

    std::string output_dir_;
};
} // namespace realsense2_camera_capture
