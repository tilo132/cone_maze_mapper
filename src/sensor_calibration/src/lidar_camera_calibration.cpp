#include <cstdio>
#include "sensor_calibration/lidar_camera_calibration.hpp"

using namespace sensor_calibration;

LidarCameraCalibrationNode::LidarCameraCalibrationNode(): Node("lidar_camera_calibration_node")
{
  this->camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
  // Parameters
  this->declare_parameter<std::string>("sub_topic_lidar", "velodyne_points");
  this->declare_parameter<std::string>("sub_topic_camera", "image_raw/compressed");
  this->declare_parameter<std::string>("sub_topic_camera_info", "camera_info");
  this->declare_parameter<std::string>("pub_topic_transform", "calibration/transform");
  this->declare_parameter<std::string>("pub_topic_image", "calibration/image/compressed");
  this->declare_parameter<std::string>("pub_topic_pointcloud", "calibration/segmented");
  this->declare_parameter<std::string>("pub_topic_marker", "calibration/marker");

  this->declare_parameter<int>("synchronizer_queue_size", 10);
  // max eps angle between the normal of the plane detected by the camera and the segmented pointcloud plane
  this->declare_parameter<double>("max_eps_angle", 0.3);
  // the distnace points can be away from the segmented plane to be counted as inliers
  this->declare_parameter<double>("segmentation_threshold", 0.005);
  // multiplier for the distance points can be away from the center of the board detected by the camera
  this->declare_parameter<double>("max_distance_factor", 2);

  this->declare_parameter<int>("sample_buffer_size", 10);

  this->declare_parameter<int>("board_width", 296);
  this->declare_parameter<int>("board_height", 210);
  this->declare_parameter<int>("delta_qr_x", 217);
  this->declare_parameter<int>("delta_qr_y", 137);
  this->declare_parameter<int>("qr_size", 30);

  //
  // ARUCO
  //

  // Calculate Corners of QR Codes
  // 
  // Markers order:
  // 0-------1
  // |       |
  // |   C   |
  // |       |
  // 3-------2

  // WARNING: IDs are in different order:
  // Marker 0 -> aRuCo ID: 1
  // Marker 1 -> aRuCo ID: 2
  // Marker 2 -> aRuCo ID: 4
  // Marker 3 -> aRuCo ID: 3

  // dict is cv Ptr of aruco dictionary
  this->aruco_dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000));

  // adjust to store 4 QR codes
  this->qr_corners.resize(4);

  // divide by 2 to calculate from the center of the board.
  // delta_qr_x is the distance between the center of 2 qr_codes. 
  // divided by 2 its the distance from the center of the board.
  // factor of 1/1000 -> convert from mm to m
  const float delta_qr_x = ((float)this->get_parameter("delta_qr_x").as_int())/2.0;
  const float delta_qr_y = ((float)this->get_parameter("delta_qr_y").as_int())/2.0;
  const float qr_size = ((float)this->get_parameter("qr_size").as_int())/2.0;

  for(uint8_t i = 0; i < 4; ++i) {
    // QR Code
    float _x = i%3==0?-1.0:1.0;
    float _y = i<2?-1.0:1.0;

    //adjust to store 4 corners of the QR code
    this->qr_corners[i].resize(4);
    for(uint8_t j = 0; j < 4; ++j){
      float _x2 = j%3==0?-1.0:1.0;
      float _y2 = j<2?-1.0:1.0;
      this->qr_corners[i][j] = cv::Point3f(
        delta_qr_x * _x + qr_size * _x2,
        delta_qr_y * _y + qr_size * _y2,
        0.0
      );
    }
  }

  const std::vector<int> qr_ids = {1, 2, 4, 3};
  this->aruco_board = cv::aruco::Board::create(
    qr_corners,
    this->aruco_dictionary,
    qr_ids
  );

  this->aruco_parameters = cv::makePtr<cv::aruco::DetectorParameters>();
  this->aruco_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

  //
  // ROS2
  //

  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  // ApproximateTimeSynchronizer / Subscriber
  this->sub_lidar.subscribe(
    this, 
    this->get_parameter("sub_topic_lidar").as_string(),
    rmw_qos_profile_sensor_data
  );
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic %s", this->get_parameter("sub_topic_lidar").as_string().c_str());
  this->sub_camera.subscribe(
    this, 
    this->get_parameter("sub_topic_camera").as_string(),
    rmw_qos_profile_sensor_data
  );
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic %s", this->get_parameter("sub_topic_camera").as_string().c_str());
  this->sub_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    this->get_parameter("sub_topic_camera_info").as_string(),
    default_qos,
    std::bind(&LidarCameraCalibrationNode::camera_info_callback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic %s", this->get_parameter("sub_topic_camera_info").as_string().c_str());
  
  this->lidar_camera_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage>>>(
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage>(
      this->get_parameter("synchronizer_queue_size").as_int() // queue size
    ),
    this->sub_lidar,
    this->sub_camera
  );
  this->lidar_camera_sync->registerCallback(std::bind(&LidarCameraCalibrationNode::lidar_camera_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Publisher
  this->pub_transform = this->create_publisher<geometry_msgs::msg::TransformStamped>(
    this->get_parameter("pub_topic_transform").as_string(),
    10
  );
  this->pub_image = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    this->get_parameter("pub_topic_image").as_string(),
    default_qos
  );
  this->pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("pub_topic_pointcloud").as_string(),
    default_qos
  );
  this->pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    this->get_parameter("pub_topic_marker").as_string(),
    10
  );
}

void LidarCameraCalibrationNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if(this->camera_info_received) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "LidarCameraCalibrationNode::camera_info_callback");
  this->camera_info = *msg;

  // Camera Matrix
  this->camera_matrix.at<float>(0, 0) = this->camera_info.k[0];
  this->camera_matrix.at<float>(0, 1) = this->camera_info.k[1];
  this->camera_matrix.at<float>(0, 2) = this->camera_info.k[2];
  this->camera_matrix.at<float>(1, 0) = this->camera_info.k[3];
  this->camera_matrix.at<float>(1, 1) = this->camera_info.k[4];
  this->camera_matrix.at<float>(1, 2) = this->camera_info.k[5];
  this->camera_matrix.at<float>(2, 0) = this->camera_info.k[6];
  this->camera_matrix.at<float>(2, 1) = this->camera_info.k[7];
  this->camera_matrix.at<float>(2, 2) = this->camera_info.k[8];

  this->dist_coeffs = cv::Mat(1, this->camera_info.d.size(), CV_32F);
  for (size_t i = 0; i < this->camera_info.d.size(); i++){
    dist_coeffs.at<float>(0, i) = this->camera_info.d[i];
  }

  this->camera_info_received = true;
}

void LidarCameraCalibrationNode::lidar_camera_callback(
  const sensor_msgs::msg::PointCloud2::ConstPtr& lidar_msg,
  const sensor_msgs::msg::CompressedImage::ConstPtr& camera_msg
)
{
  RCLCPP_INFO(this->get_logger(), "LidarCameraCalibrationNode::lidar_camera_callback");
  RCLCPP_INFO(this->get_logger(), "lidar_msg->header.frame_id: %s", lidar_msg->header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_msg->header.frame_id: %s", camera_msg->header.frame_id.c_str());

  // Check if camera info is received
  if (!this->camera_info_received) {
    RCLCPP_WARN(this->get_logger(), "Camera info is not received yet");
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array_msg;

  //
  // CAMERA
  //

  cv::Mat camera_image = cv::imdecode(cv::Mat(camera_msg->data), 1);

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  cv::aruco::detectMarkers(
    camera_image,
    this->aruco_dictionary,
    marker_corners,
    marker_ids,
    this->aruco_parameters
  );

  // Draw detected markers
  cv::aruco::drawDetectedMarkers(
    camera_image, 
    marker_corners, 
    marker_ids
  );

  if (marker_ids.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "Not enough markers detected");
    return;
  }
  // print number of detected markers
  RCLCPP_INFO(this->get_logger(), "Detected %d markers", (int)marker_ids.size());
  //camera matrix to 3x3 matrix

  // Estimate poses
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    marker_corners,
    0.05, 
    this->camera_matrix,
    this->dist_coeffs,
    rvecs,
    tvecs
  );
  // draw axis for each marker
  cv::Vec3d rvec, tvec, rvec_sin, rvec_cos;
  for(size_t i=0; i<marker_ids.size(); i++){
    cv::drawFrameAxes(
      camera_image, 
      this->camera_matrix, 
      this->dist_coeffs, 
      rvecs[i], 
      tvecs[i], 
      0.1
    );
    tvec += tvecs[i];
    rvec_sin[0] += std::sin(rvecs[i][0]);
    rvec_sin[1] += std::sin(rvecs[i][1]);
    rvec_sin[2] += std::sin(rvecs[i][2]);
    rvec_cos[0] += std::cos(rvecs[i][0]);
    rvec_cos[1] += std::cos(rvecs[i][1]);
    rvec_cos[2] += std::cos(rvecs[i][2]);
  }
  tvec /= (float)marker_ids.size();
  rvec_sin /= (float)marker_ids.size();  // Average sin
  rvec_cos /= (float)marker_ids.size();  // Average cos
  rvec[0] = std::atan2(rvec_sin[0], rvec_cos[0]);
  rvec[1] = std::atan2(rvec_sin[1], rvec_cos[1]);
  rvec[2] = std::atan2(rvec_sin[2], rvec_cos[2]);

  // Convert the Rvec and Tvec to a full transformation matrix (4th row will be kept missing)
  cv::Mat rvec_mat(3, 3, cv::DataType<float>::type);
  cv::Rodrigues(rvec, rvec_mat);
  cv::Mat tvec_mat = cv::Mat(tvec);
  cv::Mat transformation_mat = cv::Mat::eye(3, 4, CV_32F);
  rvec_mat.col(0).copyTo(transformation_mat.col(0));
  rvec_mat.col(1).copyTo(transformation_mat.col(1));
  rvec_mat.col(2).copyTo(transformation_mat.col(2));
  tvec_mat.col(0).copyTo(transformation_mat.col(3));

  // transform QR Code centers to world frame using the transformation matrix
  std::vector<cv::Point3f> qr_centers;
  for(uint8_t i = 0; i < 4; ++i) {
    qr_centers.push_back(
      (qr_corners[i][0]+qr_corners[i][2])/2.0
    );
  }
  std::vector<cv::Point3f> qr_centers_world;
  cv::transform(qr_centers, qr_centers_world, transformation_mat);
  // convert the global centers to pcl space
  for (uint8_t i = 0; i < 4; ++i) {
    float _x = qr_centers_world[i].x;
    float _y = qr_centers_world[i].y;
    float _z = qr_centers_world[i].z;
    qr_centers_world[i].x = _z;
    qr_centers_world[i].y = _x;
    qr_centers_world[i].z = _y;
  }

  // log the QR Code centers in world frame
  for(uint8_t i = 0; i < 4; ++i) {
    RCLCPP_INFO(this->get_logger(), "QR Code %d: %f %f %f", i, qr_centers_world[i].x, qr_centers_world[i].y, qr_centers_world[i].z);
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = lidar_msg->header.frame_id;
    marker_msg.header.stamp = lidar_msg->header.stamp;
    marker_msg.ns = "qrcode";
    marker_msg.id = i;
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.position.x = qr_centers_world[i].x;
    marker_msg.pose.position.y = qr_centers_world[i].y;
    marker_msg.pose.position.z = qr_centers_world[i].z;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_array_msg.markers.push_back(marker_msg);
  }

  // calculate the position and normal vector of the board plane
  cv::Point3f board_normal = (
    qr_centers_world[3] - qr_centers_world[0]
  ).cross(
    qr_centers_world[1] - qr_centers_world[0]
  );
  if(this->normal_camera.size() > this->get_parameter("sample_buffer_size").as_int()){
    normal_camera.erase(normal_camera.begin());
  }
  this->normal_camera.push_back(board_normal);

  cv::Point3f board_center = (
    qr_centers_world[0] + qr_centers_world[1] + qr_centers_world[2] + qr_centers_world[3]
  ) / 4.0;
  if(this->center_camera.size() > this->get_parameter("sample_buffer_size").as_int()){
    center_camera.erase(center_camera.begin());
  }
  this->center_camera.push_back(board_center);

  cv::Point3f avg_center_camera, avg_normal_camera;
  for (auto &p : this->center_camera) {
    avg_center_camera += p;
  }
  avg_center_camera /= this->center_camera.size();
  for (auto &p : this->normal_camera) {
    avg_normal_camera += p;
  }
  avg_normal_camera /= this->normal_camera.size();
  {
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = lidar_msg->header.frame_id;
    marker_msg.header.stamp = lidar_msg->header.stamp;
    marker_msg.ns = "board";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.position.x = avg_center_camera.x;
    marker_msg.pose.position.y = avg_center_camera.y;
    marker_msg.pose.position.z = avg_center_camera.z;
    marker_msg.pose.orientation.x = avg_normal_camera.x;
    marker_msg.pose.orientation.y = avg_normal_camera.y;
    marker_msg.pose.orientation.z = avg_normal_camera.z;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_array_msg.markers.push_back(marker_msg);
  }

  RCLCPP_INFO(this->get_logger(), "Estimated board center: %f %f %f", board_center.x, board_center.y, board_center.z);
  RCLCPP_INFO(this->get_logger(), "Estimated board normal: %f %f %f", board_normal.x, board_normal.y, board_normal.z);

  //
  // LIDAR
  //

  pcl::PointCloud<VelodynePoint>::Ptr lidar_cloud(new pcl::PointCloud<VelodynePoint>);
  pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

  // From here it a few assumptions are made:
  // - the lidar and camera are almost angular aligned
  // - the lidar and camera are not spaced too far apart
  // these assumptions are used to filter and segment the lidar data by assuming,
  // that the camera world space is not too far away from the lidar world space

  // Filter out points that are too far away from the estimated markers / board plane
  
  // uses the greater value of the board dimensions
  const int board_width = this->get_parameter("board_width").as_int();
  const int board_height = this->get_parameter("board_height").as_int();
  const float block_radius = ((float)std::max(board_width, board_height))/2000.0*(float)this->get_parameter("max_distance_factor").as_double();
  
  pcl::PassThrough<VelodynePoint> pass;
  pass.setInputCloud(lidar_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(
    board_center.x - block_radius,
    board_center.x + block_radius
  );
  pass.filter(*lidar_cloud);

  pass.setInputCloud(lidar_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(
    board_center.y - block_radius,
    board_center.y + block_radius
  );
  pass.filter(*lidar_cloud);

  pass.setInputCloud(lidar_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(
    board_center.z - block_radius,
    board_center.z + block_radius
  );
  pass.filter(*lidar_cloud);

  // SAC segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<VelodynePoint> seg;
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setDistanceThreshold((float)this->get_parameter("segmentation_threshold").as_double());
  seg.setMethodType(pcl::SAC_RANSAC);
  //convert board_normal from Point3f to Eigen::Vector3f
  Eigen::Vector3f board_normal_eigen = Eigen::Vector3f(
    board_normal.x,
    board_normal.y,
    board_normal.z
  );
  seg.setAxis(board_normal_eigen);
  seg.setEpsAngle((float)this->get_parameter("max_eps_angle").as_double());
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(100);
  seg.setInputCloud(lidar_cloud);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No inliers found");
  }else{
    cv::Mat translation_vector = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat rotation_vector = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat rotation_matrix;
    cv::Point3f lidar_normal, lidar_center;
    // calculate the position and normal vector of the board plane
    lidar_normal = cv::Point3f(
      coefficients->values[0],
      coefficients->values[1],
      coefficients->values[2]
    );
    if(this->normal_lidar.size() > this->get_parameter("sample_buffer_size").as_int()) {
      this->normal_lidar.erase(this->normal_lidar.begin());
    }
    this->normal_lidar.push_back(lidar_normal);

    // average the inliers to get the board center
    lidar_center = cv::Point3f(0.0, 0.0, 0.0);
    for(uint32_t i = 0; i < inliers->indices.size(); ++i) {
      lidar_center.x += lidar_cloud->points[inliers->indices[i]].x;
      lidar_center.y += lidar_cloud->points[inliers->indices[i]].y;
      lidar_center.z += lidar_cloud->points[inliers->indices[i]].z;
    }
    lidar_center.x /= inliers->indices.size();
    lidar_center.y /= inliers->indices.size();
    lidar_center.z /= inliers->indices.size();
    if(this->center_lidar.size() > this->get_parameter("sample_buffer_size").as_int()) {
      this->center_lidar.erase(this->center_lidar.begin());
    }

    cv::Point3f avg_center_lidar, avg_normal_lidar;
    for (auto &p : this->center_lidar) {
      avg_center_lidar += p;
    }
    avg_lidar_center /= this->center_lidar.size();
    for (auto &p : this->normal_lidar) {
      avg_normal_lidar += p;
    }
    avg_normal_lidar /= this->normal_lidar.size();


    this->center_lidar.push_back(lidar_center);
    {
      visualization_msgs::msg::Marker marker_msg;
      marker_msg.header.frame_id = lidar_msg->header.frame_id;
      marker_msg.header.stamp = lidar_msg->header.stamp;
      marker_msg.ns = "lidar";
      marker_msg.id = 0;
      marker_msg.type = visualization_msgs::msg::Marker::ARROW;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.pose.position.x = avg_center_lidar.x;
      marker_msg.pose.position.y = avg_center_lidar.y;
      marker_msg.pose.position.z = avg_center_lidar.z;
      marker_msg.pose.orientation.x = avg_normal_lidar.x;
      marker_msg.pose.orientation.y = avg_normal_lidar.y;
      marker_msg.pose.orientation.z = avg_normal_lidar.z;
      marker_msg.pose.orientation.w = 1.0;
      marker_msg.scale.x = 0.1;
      marker_msg.scale.y = 0.1;
      marker_msg.scale.z = 0.1;
      marker_msg.color.a = 1.0;
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
      marker_array_msg.markers.push_back(marker_msg);
    }

    // calculate rotation and translation from camera to lidar
    cv::Point3f rotation_axis = avg_normal_lidar.cross(avg_normal_camera);
    // normalize rotation_axis
    rotation_axis *= 1.0/cv::norm(rotation_axis);

    float rotation_angle = std::acos(avg_normal_lidar.dot(avg_normal_camera));

    rotation_vector.at<float>(0, 0) = rotation_axis.x*rotation_angle;
    rotation_vector.at<float>(1, 0) = rotation_axis.y*rotation_angle;
    rotation_vector.at<float>(2, 0) = rotation_axis.z*rotation_angle;
    cv::Rodrigues(
      rotation_vector,
      rotation_matrix
    );

    translation_vector.at<float>(0, 0) = avg_center_lidar.x - avg_center_camera.x;
    translation_vector.at<float>(1, 0) = avg_center_lidar.y - avg_center_camera.y;
    translation_vector.at<float>(2, 0) = avg_center_lidar.z - avg_center_camera.z;
  
    // convert filtered lidar cloud (inliers) to ros pointcloud2 and publish it
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::PointCloud<VelodynePoint>::Ptr lidar_cloud_inliers(new pcl::PointCloud<VelodynePoint>);
    pcl::copyPointCloud(*lidar_cloud, inliers->indices, *lidar_cloud_inliers);
    pcl::toROSMsg(*lidar_cloud_inliers, *pointcloud_msg);
    pointcloud_msg->header.frame_id = lidar_msg->header.frame_id;
    this->pub_pointcloud->publish(*pointcloud_msg);
  
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = lidar_msg->header.stamp;
    transform_msg.header.frame_id = camera_msg->header.frame_id;
    transform_msg.child_frame_id = lidar_msg->header.frame_id;
    transform_msg.transform.translation.x = translation_vector.at<float>(0, 0);
    transform_msg.transform.translation.y = translation_vector.at<float>(1, 0);
    transform_msg.transform.translation.z = translation_vector.at<float>(2, 0);
    transform_msg.transform.rotation.x = rotation_matrix.at<float>(0, 0);
    transform_msg.transform.rotation.y = rotation_matrix.at<float>(1, 0);
    transform_msg.transform.rotation.z = rotation_matrix.at<float>(2, 0);
    transform_msg.transform.rotation.w = 1.0;
    this->pub_transform->publish(transform_msg);
  }
  //
  // Publish
  //

  RCLCPP_INFO(this->get_logger(), "Publishing");

  // convert camera_image to ros compressed image and publish it
  sensor_msgs::msg::CompressedImage::SharedPtr image_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    camera_image
  ).toCompressedImageMsg();
  this->pub_image->publish(*image_msg);
  // Publish board center and normal vector
  this->pub_marker->publish(marker_array_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCameraCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
