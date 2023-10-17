#ifndef WALL_LAYER_HPP_
#define WALL_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
//#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "cat_msgs_ros/msg/walls.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Point32 (x,y,z) with operator overloading and implicit conversion to geometry_msgs::msg::Point32
class Point32 : public geometry_msgs::msg::Point32 {
public:
    Point32() : geometry_msgs::msg::Point32() {}
    Point32(double x, double y, double z) : geometry_msgs::msg::Point32() {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    Point32(geometry_msgs::msg::Point32 p) : geometry_msgs::msg::Point32() {
        this->x = p.x;
        this->y = p.y;
        this->z = p.z;
    }
    Point32 operator+(const Point32& p) const {
        return Point32(this->x + p.x, this->y + p.y, this->z + p.z);
    }
    Point32 operator-(const Point32& p) const {
        return Point32(this->x - p.x, this->y - p.y, this->z - p.z);
    }
    Point32 operator*(const double& d) const {
        return Point32(this->x * d, this->y * d, this->z * d);
    }
    Point32 operator/(const double& d) const {
        return Point32(this->x / d, this->y / d, this->z / d);
    }
    Point32 operator+=(const Point32& p) {
        this->x += p.x;
        this->y += p.y;
        this->z += p.z;
        return *this;
    }
    Point32 operator-=(const Point32& p) {
        this->x -= p.x;
        this->y -= p.y;
        this->z -= p.z;
        return *this;
    }
    Point32 operator*=(const double& d) {
        this->x *= d;
        this->y *= d;
        this->z *= d;
        return *this;
    }
    Point32 operator/=(const double& d) {
        this->x /= d;
        this->y /= d;
        this->z /= d;
        return *this;
    }
    double length() const {
        return std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
    }
    Point32 normalize() const {
        return *this / this->length();
    }
    double dot(const Point32& p) const {
        return this->x
            * p.x
            + this->y
            * p.y
            + this->z
            * p.z;
    }
    Point32 cross(const Point32& p) const {
        return Point32(
            this->y * p.z - this->z * p.y,
            this->z * p.x - this->x * p.z,
            this->x * p.y - this->y * p.x
        );
    }
};


namespace cat_nav2
{

    class WallLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        WallLayer();

        virtual void onInitialize();
        virtual void updateBounds(
            double /*robot_x*/,
            double /*robot_y*/,
            double /*robot_yaw*/,
            double* min_x,
            double* min_y,
            double* max_x,
            double* max_y
        );

        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i,
            int min_j,
            int max_i,
            int max_j
        );

        /**
         * @brief Activate this layer
         */
        virtual void activate(){
            return;
        }
        /**
         * @brief Deactivate this layer
         */
        virtual void deactivate(){
            return;
        }

        /**
         * @brief Reset this costmap
         */
        virtual void reset();

        /**
         * @brief If clearing operations should be processed on this layer or not
         */
        virtual bool isClearable() {
            return false;
        }

        /**
         * @brief Match the size of the master costmap
         */
        virtual void matchSize(){
            return;
        }


        virtual void onFootprintChanged()
        {
            return;
        }

    private:
        rclcpp::Subscription<cat_msgs_ros::msg::Walls>::SharedPtr sub_walls;
        cat_msgs_ros::msg::Walls::SharedPtr walls;
        std::vector<Point32> cones;
        
        void wallUpdate(
            cat_msgs_ros::msg::Walls::SharedPtr msg
        );

        // tf buffer
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        //bool only_ground_;

        void drawWalls();
    };

}

#endif
