from typing import List
import cv2
import time
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
#from rclpy.parameter import Parameter7
#Sensordaten und Camerabilder
#Meldung wo und wann ich wie stehe
#from cat_msgs_ros.msg import PolygonsStamped
from sensor_msgs.msg import CompressedImage, PointCloud2, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from cv_bridge import CvBridge
from rclpy.subscription import Subscription
from std_msgs.msg import String
import math
from PIL import Image, ImageDraw
from tf2_geometry_msgs import do_transform_point
import tf2_ros

class QRCode(Node):

    camera_matrix=None
    bbox = None
    dist_coeffs=None
    trans = None

    FRAME_BASE  = "base_link"
    FRAME_LIDAR = "velodyne_link"
    FRAME_CAMERA = "camera_link"   
    
     # Quality of Service
    QoS = rclpy.qos.QoSProfile(
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        depth=1,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST
    )


    #Subscribtion/ kommunikation mit ros
    #alles zur kamera
    def __init__(self):
        super().__init__("QRCode")
        self.create_subscription(
            msg_type=CompressedImage,
            topic='image_raw/compressed',
            callback= self.image_callback,
            qos_profile=self.QoS
        )
        self.declare_parameter('qr_code_side_length', 0.09)
        
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, self.QoS)
        self.pose_pub = self.create_publisher(PoseStamped, "poi", self.QoS)
        
        #Einbindung des Lidars ist nicht notwendig da wir Camera Matrix verwenden um 3D Position von dem QR code berechnen. Erhöht die Schnelligkeit 
        #self.subLidar = self.create_subscription(PointCloud2, 'velodyne_points', self.onLidar, self.QoS)

        # CV-Bridge for image conversion
        self.cv_bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        print("Qr_Code Initialized")
    
    # Was er tun soll wenn Subscription eingegangen
    def image_callback(self, msg):
        if self.trans is None:
            if not self.updateTransform():
                print("cant find Transform")
                return
        
    
        image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        data, bbox, straight_qrcode = self.detector.detectAndDecode(image)
        #bbox2 = self.detector.detect(image)
        if len(data) > 0:
            print(data)
            #image = self.draw_rectangle(image, bbox)
            #self.display_image(image)

            if self.camera_matrix is None or self.dist_coeffs is None:
                return
            
            if bbox is None or len(bbox) == 0:
                return

            # In Opencv Coordinate System x right, y down, z into the screen
            qr_code_side_length = self.get_parameter('qr_code_side_length').value / 2.0
            object_points = np.array([
                [-qr_code_side_length, -qr_code_side_length, 0],
                [qr_code_side_length, -qr_code_side_length, 0],
                [qr_code_side_length ,qr_code_side_length, 0],
                [-qr_code_side_length ,qr_code_side_length, 0]],
                dtype=np.float32)

            image_points = bbox
            object_pose = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                None,
                None,
                1,
                cv2.SOLVEPNP_P3P
            )
            _, rvec, tvec = object_pose

            if tvec is None:
                return None

            p = PointStamped()
            p.header.frame_id = self.FRAME_BASE
            p.point.x = float(tvec[2])
            p.point.y = float(-tvec[0])
            p.point.z = float(-tvec[1])
            p = do_transform_point(p, self.trans)
                       
            #Publish the position of the Qrcode
            ps = PoseStamped()
            ps.pose.position = p.point
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = self.FRAME_BASE
            self.pose_pub.publish(ps)
            print(ps.pose.position)

            
            #((npoints >= 4) || (npoints == 3 && flags == SOLVEPNP_ITERATIVE && useExtrinsicGuess) || (npoints >= 3 && flags == SOLVEPNP_SQPNP)) 
            #&& npoints == std::max(ipoints.checkVector(2, CV_32F), ipoints.checkVector(2, CV_64F))
#           print(object_pose)
            # distance= math.sqrt(tvec[0]**2+tvec[1]**2+tvec[2]**2)

            # print("Hat funktioniert:", distance)

            
    def updateTransform(self):
        try:
            self.trans = self.tf_buffer.lookup_transform(self.FRAME_CAMERA, self.FRAME_BASE, rclpy.time.Time())
            return True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            print(f'Unable to find the transformation')
            return False
            
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)


    #draw Rectangle         Um herauszufinden ob wir wissen wo sich der QR Code im Raum befindet mit den bestimmten Pixeln
        # create rectangle image
    def draw_rectangle(self, image, bbox):
            #if len(bbox) == 4:

        #print(x1,y1,x3,y3)
        bbox = np.int32(bbox)
        x1, y1 = bbox[0][0]
        x2, y2 = bbox[0][1]
        x3, y3 = bbox[0][2]
        x4, y4 = bbox[0][3]
        cv2.rectangle(image, (x1, y1), (x3, y3), (0, 0, 255), 1)

        return image
    def display_image(self, image):
        cv2.imshow("QRCode", image)
        cv2.waitKey(1)





"""
        self.trans = (
                np.array([
                    self.trans_msg.transform.translation.x,
                    self.trans_msg.transform.translation.y,
                    self.trans_msg.transform.translation.z
                ], np.float),
                np.array([
                    rot[0],
                    rot[1],
                    rot[2]
                ], np.float)
            )
        """
        




def main(args=None):
    rclpy.init(args=args)
    my_node= QRCode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
        main()
