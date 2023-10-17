/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "cat_nav2/wall_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cat_nav2
{

  WallLayer::WallLayer(){
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void WallLayer::onInitialize()
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    RCLCPP_INFO(logger_, "Initialized WallLayer");

    //declareParameter("only_ground", rclcpp::ParameterValue(false));
    //node->get_parameter(name_ + "." + "only_ground", only_ground_);
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    this->current_ = true;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->sub_walls = node->create_subscription<cat_msgs_ros::msg::Walls>(
      "/walls",
      1,
      std::bind(&WallLayer::wallUpdate, this, std::placeholders::_1)
    );
    RCLCPP_INFO(logger_, "WallLayer: Subscribed to /walls");
  }

  // This method is called when new message is received from the topic
  // specified in the subscriber constructor.
  // It updates the walls variable with the new message.
  void WallLayer::wallUpdate(
      cat_msgs_ros::msg::Walls::SharedPtr msg)
  {
  	//RCLCPP_INFO(logger_, "WallLayer: Updating Walls");   
    this->walls = msg;

    //RCLCPP_INFO(logger_, "WallLayer: Received %ld Walls", this->walls->walls.size()/2);

    //empty transformed cones vec
    this->cones.clear();

    const auto global_frame = layered_costmap_->getGlobalFrameID();
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform(
        global_frame,
        this->walls->header.frame_id,
        tf2::TimePointZero
        //tf2::TimePoint(std::chrono::seconds(5))
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(logger_, "%s", ex.what());
      return;
    }
    for (auto cone: this->walls->cones) {
      geometry_msgs::msg::Point casted_cone;
      casted_cone.x = (double)cone.x;
      casted_cone.y = (double)cone.y;
      casted_cone.z = (double)cone.z;
      geometry_msgs::msg::Point transformed_cone;
      tf2::doTransform(casted_cone, transformed_cone, transformStamped);
      geometry_msgs::msg::Point32 backcasted_cone;
      backcasted_cone.x = (float)transformed_cone.x;
      backcasted_cone.y = (float)transformed_cone.y;
      backcasted_cone.z = (float)transformed_cone.z;
      cones.push_back((Point32)backcasted_cone);
    }

    // calculate the bounds of the update
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    for (auto cone : this->cones) {
      min_x = std::max(std::min(min_x, (double)cone.x), -1000.0);
      min_y = std::max(std::min(min_y, (double)cone.y), -1000.0);
      max_x = std::min(std::max(max_x, (double)cone.x), 1000.0);
      max_y = std::min(std::max(max_y, (double)cone.y), 1000.0);
    }

    {
      // resize this layer's costmap
      // lock the costmap
      std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
      nav2_costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();

      unsigned int size_x = (unsigned int)((max_x-min_x) / master->getResolution());
      unsigned int size_y = (unsigned int)((max_y-min_y) / master->getResolution());

      this->resizeMap(
        size_x,
        size_y,
        master->getResolution(),
        min_x,
        min_y
      );
      this->resetMapToValue(/*min_x,min_y*/ 0U, 0U ,size_x,size_y, NO_INFORMATION);
    }
    //RCLCPP_INFO(logger_, "WallLayer: End Updating Walls");   
  }


  void WallLayer::reset()
  {
    /*RCLCPP_INFO(logger_, "WallLayer: Resetting");
    Costmap2D::resetMaps();
    this->walls = nullptr;
    this->current_ = true;*/
  }

  void WallLayer::updateBounds(
    double /*robot_x*/,
    double /*robot_y*/,
    double /*robot_yaw*/,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y
  ){
    //RCLCPP_INFO(logger_, "WallLayer: Updating Bounds");

    // merge bounds of master costmap and this layer and resize the master costmap if needed

    // get the bounds of the master costmap
    double m_min_x, m_min_y, m_max_x, m_max_y;
    nav2_costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();

    m_min_x = master->getOriginX();
    m_min_y = master->getOriginY();
    m_max_x = m_min_x + master->getSizeInMetersX();
    m_max_y = m_min_y + master->getSizeInMetersY();
    // lock the costmap
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    // get the bounds of this layer
    double l_min_x, l_min_y, l_max_x, l_max_y;
    l_min_x = this->getOriginX();
    l_min_y = this->getOriginY();
    l_max_x = l_min_x + this->getSizeInMetersX();
    l_max_y = l_min_y + this->getSizeInMetersY();
    
    // merge the bounds
    *min_x = std::min({m_min_x, l_min_x, *min_x});
    *min_y = std::min({m_min_y, l_min_y, *min_y});
    *max_x = std::max({m_max_x, l_max_x, *max_x});
    *max_y = std::max({m_max_y, l_max_y, *max_y});

    return;
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap gradient is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void WallLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int /*min_i*/,
    int /*min_j*/,
    int /*max_i*/,
    int /*max_j*/
  ){
    //RCLCPP_INFO(logger_, "WallLayer: Updating Costs");   
    this->drawWalls();
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    const int delta_x = (int)((master_grid.getOriginX() - this->getOriginX())/this->getResolution());
    const int delta_y = (int)((master_grid.getOriginY() - this->getOriginY())/this->getResolution());

    //RCLCPP_INFO(logger_, "WallLayer: Merging with master map %d x %d with offset (%d, %d) [%f, %f] -> [%f %f]", this->getSizeInCellsX(), this->getSizeInCellsY(), delta_x, delta_y, this->getOriginX(), this->getOriginY(), master_grid.getOriginX(), master_grid.getOriginY());

    for (unsigned int i = 0; i < this->getSizeInCellsX(); i++) {
      for (unsigned int j = 0; j < this->getSizeInCellsY(); j++) {
        unsigned char cost = this->getCost(i, j);
        //if (only_ground_ && master_grid.getCost(mx, my) == LETHAL_OBSTABLE){
        //	master_grid.setCost(mx, my, NO_INFORMATION);
        //}
        if (cost != NO_INFORMATION) {
          const int mx = i - delta_x;
          const int my = j - delta_y;
          if (mx >= (int)master_grid.getSizeInCellsX() || my >= (int)master_grid.getSizeInCellsY() || mx < 0 || my < 0) {
          	continue;
          }
          master_grid.setCost(mx, my, cost);
        }
      }
    }
    //RCLCPP_INFO(logger_, "WallLayer: End Updating Costs");   
  }

  void WallLayer::drawWalls(){
 	//RCLCPP_INFO(logger_, "WallLayer: Drawing Walls");   
    if (!enabled_)
    {
      RCLCPP_INFO(logger_, "WallLayer: Disabled");
      return;
    }
    //RCLCPP_INFO(logger_, "WallLayer: Pre Mutex");
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    //RCLCPP_INFO(logger_, "WallLayer: Post Mutex");

    // resolution: meters per cell
    const double resolution = this->getResolution();

    // draw wall as line with infinite cost onto the costmap
    for (uint16_t i = 0; i < this->walls->walls.size(); i += 2)
    {
      Point32 cone1 = this->cones[this->walls->walls[i]];
      Point32 cone2 = this->cones[this->walls->walls[i + 1]];
      Point32 v12 = cone2 - cone1;
      v12.z = 0;
      double line_length = std::sqrt(v12.x * v12.x + v12.y * v12.y);
      int num_steps = std::ceil(line_length / resolution);
      if(num_steps == 0){
      	continue;
      }
      Point32 line_step = v12 / num_steps;

      // draw the line
      for (int i = 0; i <= num_steps; i++)
      {
        Point32 point = cone1 + line_step * i;
        unsigned int mx, my;
        if (this->worldToMap(point.x, point.y, mx, my))
        {
          this->setCost(mx, my, LETHAL_OBSTACLE);
          //this->setCost(mx, my+1, LETHAL_OBSTACLE);
          //this->setCost(mx+1, my, LETHAL_OBSTACLE);
          //this->setCost(mx, my-1, LETHAL_OBSTACLE);
          //this->setCost(mx-1, my, LETHAL_OBSTACLE);
        }
      }
    }
    //RCLCPP_INFO(logger_, "WallLayer: Drawing %ld Walls", this->walls->walls.size()/2);
  }
}

// This is the macro allowing a nav2_gradient_costmap_plugin::WallLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::CostmapLayer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cat_nav2::WallLayer, nav2_costmap_2d::Layer)
