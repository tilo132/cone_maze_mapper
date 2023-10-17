import rclpy
from rclpy.node import Node
from custom_messages.msg import Polygons
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from foxglove_msgs.msg import PosesInFrame
from std_msgs.msg import Header

import cv2
import numpy as np

class PnPNode(Node):

    camera_matrix = np.array([  [1.92350898e+03, 0.00000000e+00, 5.19676053e+02],
                                [0.00000000e+00, 1.94733854e+03, 9.24457550e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist_coeffs = np.array([ 2.89891794e-01,-1.27613940e+00 ,5.27970594e-04,-4.39755096e-04,1.85063863e+00])
    objPoints = np.array([[-68.5,47.0,0.0], [-54.8,135.0,0.0], [-37.7,223.0,0.0] ,[0.0,325.0,0.0] ,[37.7,223.0,0.0], [54.8,135.0,0.0], [68.5,47.0,0.0]])

    def __init__(self):
        super().__init__('pnp')

        self.keypointSub = self.create_subscription(Polygons, 'keypoints', self.onKeypoints,10)
        self.posePub = self.create_publisher(PosesInFrame, 'poses', 10)
        self.posePub2 = self.create_publisher(PoseStamped, 'pose', 10)
     
    def onKeypoints(self, msg):
        print("Keypoints received")

        if len(msg.polygons) == 0:
            return
        poses = []
        for poly in msg.polygons:
            imgPoints = np.array([[p.x,p.y] for p in poly.points])
            success, _rotation_vector, translation_vector = cv2.solvePnP(self.objPoints, imgPoints, self.camera_matrix, self.dist_coeffs, False, cv2.SOLVEPNP_ITERATIVE)
            #print(rotation_vector, translation_vector)
            if success:
                self.posePub2.publish(PoseStamped(pose = Pose(orientation = Quaternion(), position = Point(x = translation_vector[0][0], y = translation_vector[1][0], z = translation_vector[2][0])), 
                                                header = Header(stamp =self.get_clock().now().to_msg(), frame_id = "ftf")))
                poses.append(Pose(orientation = Quaternion(), 
                            position = Point(x = translation_vector[0][0], y = translation_vector[1][0], z = translation_vector[2][0])))
        
        self.posePub.publish(PosesInFrame(poses = [Pose(orientation = Quaternion(), position = Point(x = 10.0, y = 20.0, z = 0.0))], frame_id = "ftf", timestamp = self.get_clock().now().to_msg()))

def main(args=None):
    rclpy.init(args=args)
    my_node = PnPNode()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.video_writer.release()
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()