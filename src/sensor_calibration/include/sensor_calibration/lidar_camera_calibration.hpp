#ifndef LIDAR_CAMERA_CALIBRATION_HPP
#define LIDAR_CAMERA_CALIBRATION_HPP

#define PCL_NO_PRECOMPILE

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace sensor_calibration{
    class LidarCameraCalibrationNode: public rclcpp::Node{
        public:
            LidarCameraCalibrationNode();
        private:
            // ApproximateTimeSynchronizer / Subscriber
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar;
            message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_camera;
            std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage>>> lidar_camera_sync;
            // ros subscriber for camera info
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
            
            bool camera_info_received = false;
            sensor_msgs::msg::CameraInfo camera_info;

            cv::Ptr<cv::aruco::Dictionary> aruco_dictionary;
            cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters;
            cv::Ptr<cv::aruco::Board> aruco_board;
            std::vector<std::vector<cv::Point3f>> qr_corners;

            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;

            std::vector<cv::Vec3d> center_camera;
            std::vector<cv::Vec3d> normal_camera;

            std::vector<cv::Vec3d> center_lidar;
            std::vector<cv::Vec3d> normal_lidar;

            // Callback
            void lidar_camera_callback(
                const sensor_msgs::msg::PointCloud2::ConstPtr& lidar_msg,
                const sensor_msgs::msg::CompressedImage::ConstPtr& camera_msg
            );

            void camera_info_callback(
                const sensor_msgs::msg::CameraInfo::SharedPtr msg
            );

            // Publisher
            rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_transform;
            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
            // Rviz2 Marker Publisher
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
    };
}

struct VelodynePoint{
    PCL_ADD_POINT4D
    float intensity;     ///< laser intensity reading
    uint16_t ring;  ///< laser ring number
    float time;     ///< time stamp
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)

#endif