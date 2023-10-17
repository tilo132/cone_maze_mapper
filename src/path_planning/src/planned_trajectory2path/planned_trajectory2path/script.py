import numpy as np
import rclpy
from fszhaw_msgs.msg import CurrentPosition, PlannedTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from nav_msgs.msg import Odometry as odom
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64


class Converter(Node):
	old_path=[]
	path_generated=False
	odom_received=False
	def __init__(self):
		super().__init__('converter')
		self.create_subscription(odom,'odom',self.odom_callback,10)
		self.create_subscription(PlannedTrajectory,'planned_trajectory',self.listener_callback,10)
		self.path_publisher_=self.create_publisher(PoseStamped,'goal_pose',10)
		self.velocity_publisher_=self.create_publisher(TwistStamped, 'velocity', 10)
		
	
	def odom_callback(self,msg):
		if(self.path_generated):
			return
		self.odom_received=True
		self.old_path=[msg.pose.pose.position.x,msg.pose.pose.position.y]
		

	def calculate_quaternion(self, waypoint_old, waypoint_new):
		
		euler_angle=np.arctan2(waypoint_new[1]-waypoint_old[1], waypoint_new[0]-waypoint_old[0])
		quaternion=Rotation.from_euler('xyz', [0,0,euler_angle], degrees=False).as_quat()
		return quaternion
	
	def listener_callback(self,msg):
		if(self.odom_received==False):
			return
		path_generated=True
		print("callback")
		# Publish the velocity
		twist_stamped = TwistStamped()
		twist_stamped.header.stamp = self.get_clock().now().to_msg()
		twist_stamped.header.frame_id = 'map'
		twist_stamped.twist.linear.x = msg.target_velocity
		self.velocity_publisher_.publish(twist_stamped)
		#publish the pose
		path=Path()
		path.header.stamp=self.get_clock().now().to_msg()
		path.header.frame_id='map'
		pathpoint=PoseStamped()
		pathpoint.header.stamp=self.get_clock().now().to_msg()
		pathpoint.header.frame_id='map'
		pathpoint.pose.position.x=msg.target_x
		pathpoint.pose.position.y=msg.target_y
		pathpoint.pose.position.z=0.0
		(pathpoint.pose.orientation.x,
			pathpoint.pose.orientation.y,
			pathpoint.pose.orientation.z,
			pathpoint.pose.orientation.w) = self.calculate_quaternion(self.old_path,[msg.target_x,msg.target_y])
		
		self.path_publisher_.publish(pathpoint)
		self.old_path=[msg.target_x,msg.target_y]

def main():
	rclpy.init()
	converter = Converter()
	rclpy.spin(converter)
	rclpy.shutdown()

if __name__ == '__main__':
	main()