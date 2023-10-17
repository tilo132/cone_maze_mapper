import numpy as np
import rclpy
from rclpy.action import ActionClient
from fszhaw_msgs.msg import CurrentPosition
from fs_msgs.msg import Cone
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from cat_msgs_ros.msg import Cones as CatCones
from nav_msgs.msg import  Odometry
from nav2_msgs.action import FollowPath

import numpy as np
import math

from std_msgs.msg import Float64

class CatConverter(Node):
    old_path=[]
    def __init__(self):
        super().__init__('path_planning_cat_converter')

        # build custom qos
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            Odometry,
            'odom',
            self.listener_odom_callback,
            10
        )
        self.create_subscription(
            CatCones,
            'cones/cones',
            self.listener_cones_callback,
            qos_profile
        )
        self.create_subscription(
            Path,
            'planned_path',
            self.listener_path_callback,
            10
        )

        self.pub = {
            "path": self.create_publisher(
                CurrentPosition,
                'current_position',
                10
            ),
            "cone": self.create_publisher(
                Cone,
                'cone',
                10
            )
        }

        self.follow_path_action_client = ActionClient(self, FollowPath, 'follow_path')
        
    def listener_odom_callback(self, msg: Odometry):
        # log info
        self.get_logger().debug('Received odom message')
        casted_msg=CurrentPosition()
        
        casted_msg.vehicle_position_x = msg.pose.pose.position.x
        casted_msg.vehicle_position_y = msg.pose.pose.position.y

        x, y, w, z = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        casted_msg.yaw = math.atan2(2.0*(y*z + w*x), w*w -x*x - y*y + z*z)

        casted_msg.vehicle_velocity=np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)

        self.pub["path"].publish(casted_msg)

    def listener_cones_callback(self, msg: CatCones):
        # log info
        self.get_logger().debug('Received cones message')
        for cat_cone in msg.cones:
            cone=Cone()
            cone.location.x=cat_cone.pose.position.x
            cone.location.y=cat_cone.pose.position.y
            cone.location.z=cat_cone.pose.position.z
            if(cat_cone.color==0):
                cone.color=0 # blau
            elif(cat_cone.color==4):
                cone.color=1 #yellow
            else:
                cone.color=4 # unknown
            #2 orange big #3 orange_small
            self.pub["cone"].publish(cone)
    
    def listener_path_callback(self, msg: Path):
        """
        Gets called when a new path was calculated.
        Calls the nav2 follow_path action server with the new path and aborts the current path.
        """
        # log info
        self.get_logger().info('Passing new path to PathFollower')
        # abort current path
        self.follow_path_action_client.wait_for_server()
        #new path
        new_path = FollowPath.Goal()
        new_path.path = msg
        new_path.controller_id = "FollowPath"
        new_path.goal_checker_id = "general_goal_checker"
        self.follow_path_action_client.send_goal_async(new_path)


def main():
    rclpy.init()
    converter = CatConverter()
    rclpy.spin(converter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()