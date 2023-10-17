// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "cat_node/sensors/imu.hpp"

#include <memory>
#include <string>
#include <utility>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using robotis::turtlebot3::sensors::Imu;

Imu::Imu(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & imu_topic_name,
  const std::string & mag_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);
  mag_pub_ = nh->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, this->qos_);

  is_calibrating_acc = true;
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create imu publisher");
}

void Imu::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

  imu_msg->header.frame_id = this->frame_id_;
  imu_msg->header.stamp = now;

  imu_msg->orientation.w = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_w.addr,
    extern_control_table.imu_orientation_w.length);

  imu_msg->orientation.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_x.addr,
    extern_control_table.imu_orientation_x.length);

  imu_msg->orientation.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_y.addr,
    extern_control_table.imu_orientation_y.length);

  imu_msg->orientation.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_z.addr,
    extern_control_table.imu_orientation_z.length);

  imu_msg->angular_velocity.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_x.addr,
    extern_control_table.imu_angular_velocity_x.length);

  imu_msg->angular_velocity.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_y.addr,
    extern_control_table.imu_angular_velocity_y.length);

  imu_msg->angular_velocity.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_z.addr,
    extern_control_table.imu_angular_velocity_z.length);

  geometry_msgs::msg::PointStamped accel;
  accel.point.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_x.addr,
    extern_control_table.imu_linear_acceleration_x.length);

  accel.point.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_y.addr,
    extern_control_table.imu_linear_acceleration_y.length);

  accel.point.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_z.addr,
    extern_control_table.imu_linear_acceleration_z.length);


  // geometry_msgs::msg::PointStamped accel_calib;
  // tf2::doTransform(accel, accel_calib, transform);
  // imu_msg->linear_acceleration.x = accel.point.x - a_x;
  // imu_msg->linear_acceleration.y = accel.point.y - a_y;
  // imu_msg->linear_acceleration.z = accel.point.z - a_z;

  // imu_msg->linear_acceleration.x = dxl_sdk_wrapper->get_data_from_device<float>(
  //   extern_control_table.imu_linear_acceleration_x.addr,
  //   extern_control_table.imu_linear_acceleration_x.length) + a_x_offset;

  // imu_msg->linear_acceleration.y = dxl_sdk_wrapper->get_data_from_device<float>(
  //   extern_control_table.imu_linear_acceleration_y.addr,
  //   extern_control_table.imu_linear_acceleration_y.length) + a_y_offset;

  // imu_msg->linear_acceleration.z = dxl_sdk_wrapper->get_data_from_device<float>(
  //   extern_control_table.imu_linear_acceleration_z.addr,
    // extern_control_table.imu_linear_acceleration_z.length) + a_z_offset;

  imu_msg->linear_acceleration_covariance[0] = 0.001;
  imu_msg->linear_acceleration_covariance[4] = 0.001;
  imu_msg->linear_acceleration_covariance[8] = 0.001;
  imu_msg->angular_velocity_covariance[0] = 0.05;
  imu_msg->angular_velocity_covariance[4] = 0.05;
  imu_msg->angular_velocity_covariance[8] = 0.05;
  imu_msg->orientation_covariance[0] = 0.01;
  imu_msg->orientation_covariance[4] = 0.01;
  imu_msg->orientation_covariance[8] = 0.01;
  
  // imu_msg->orientation_covariance[0] = 2.56424074e-06;
  // 
  // imu_msg->orientation_covariance[1] = -9.22385845e-07;
  // imu_msg->orientation_covariance[2] = -1.14928187e-04;
  // imu_msg->orientation_covariance[3] = -9.22385845e-07;
  // 
  // imu_msg->orientation_covariance[4] = 4.93194150e-06;
// 
  // imu_msg->orientation_covariance[5] = 2.34791720e-04;
  // imu_msg->orientation_covariance[6] = -1.14928187e-04;
  // imu_msg->orientation_covariance[7] = 2.34791720e-04;
  // 
  // imu_msg->orientation_covariance[8] = 3.00922438e-02;
// 
// 
  // imu_msg->angular_velocity_covariance[0] = 2.90599624e-05;
// 
  // imu_msg->angular_velocity_covariance[1] = -9.33434446e-06;
  // imu_msg->angular_velocity_covariance[2] = -1.75419239e-04;
  // imu_msg->angular_velocity_covariance[3] = -9.33434446e-06;
  // 
  // imu_msg->angular_velocity_covariance[4] = 3.85396099e-05;
// 
  // imu_msg->angular_velocity_covariance[5] = 3.36432718e-05;
  // imu_msg->angular_velocity_covariance[6] = -1.75419239e-04;
  // imu_msg->angular_velocity_covariance[7] = 3.36432718e-05;
// 
  // imu_msg->angular_velocity_covariance[8] = 1.14682776e-02;
// 
// 
  // imu_msg->linear_acceleration_covariance[0] = 0.00618127;
// 
  // imu_msg->linear_acceleration_covariance[1] = -0.00064145;
  // imu_msg->linear_acceleration_covariance[2] = 0.00092676;
  // imu_msg->linear_acceleration_covariance[3] = -0.00064145;
  // 
  // imu_msg->linear_acceleration_covariance[4] = 0.0057054;
// 
  // imu_msg->linear_acceleration_covariance[5] = -0.00075595;
  // imu_msg->linear_acceleration_covariance[6] = 0.00092676;
  // imu_msg->linear_acceleration_covariance[7] = -0.00075595;
  // 
  // imu_msg->linear_acceleration_covariance[8] = 0.00592891;
 
 

  auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

  mag_msg->header.frame_id = this->frame_id_;
  mag_msg->header.stamp = now;

  mag_msg->magnetic_field.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_x.addr,
    extern_control_table.imu_magnetic_x.length);

  mag_msg->magnetic_field.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_y.addr,
    extern_control_table.imu_magnetic_y.length);

  mag_msg->magnetic_field.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_z.addr,
    extern_control_table.imu_magnetic_z.length);

  //Accelerometer Calibration
  if (is_calibrating_acc) {
    if (calib_a_x.size() >= calib_size) {
      is_calibrating_acc = false;
      
      a_x = std::accumulate(calib_a_x.begin(), calib_a_x.end(), 0.0) / calib_size;
      a_y = std::accumulate(calib_a_y.begin(), calib_a_y.end(), 0.0) / calib_size;
      a_z = std::accumulate(calib_a_z.begin(), calib_a_z.end(), 0.0) / calib_size - 9.81;

      // TF2 Quaternion between a_offfset and g
      // tf2::Vector3 a(x, y, z);
      // tf2::Vector3 b(0, 0, 9.81);


      // tf2Scalar angle = tf2::tf2Angle(a, b);
      // tf2::Vector3 axis = tf2::tf2Cross(a, b).normalize();
// 
      // tf2::Quaternion q(axis, angle);
      // transform.transform.rotation = tf2::toMsg(q);
      
      // geometry_msgs::msg::PointStamped accel;
      // accel.point.x = x;
      // accel.point.y = y;
      // accel.point.z = z;
      
      // geometry_msgs::msg::PointStamped accel_calib;
      // tf2::doTransform(accel, accel_calib, transform);
  
      RCLCPP_INFO(nh_->get_logger(), "Finished Calibration original was (%f, %f, %f)", a_x,a_y,a_z);
      calib_a_x.clear();
      calib_a_y.clear();
      calib_a_z.clear();


    } else {
      calib_a_x.push_back(accel.point.x);
      calib_a_y.push_back(accel.point.y);
      calib_a_z.push_back(accel.point.z);
      return;
    }
  }

  calib_a_x.push_back(accel.point.x - a_x);
  calib_a_y.push_back(accel.point.y - a_y);
  calib_a_z.push_back(accel.point.z - a_z);

  if (calib_a_x.size() > 20) {
    calib_a_x.erase(calib_a_x.begin());
    calib_a_y.erase(calib_a_y.begin());
    calib_a_z.erase(calib_a_z.begin());
  }

  double x = std::accumulate(calib_a_x.begin(), calib_a_x.end(), 0.0) / 20;
  double y = std::accumulate(calib_a_y.begin(), calib_a_y.end(), 0.0) / 20;
  double z = std::accumulate(calib_a_z.begin(), calib_a_z.end(), 0.0) / 20;

  imu_msg->linear_acceleration.x = x;
  imu_msg->linear_acceleration.y = y;
  imu_msg->linear_acceleration.z = z;

  imu_pub_->publish(std::move(imu_msg));
  mag_pub_->publish(std::move(mag_msg));
}
