import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import tf2_ros
from tf_transformations import euler_from_quaternion
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs.msg import CameraInfo
import collections

class LidarOverlay(Node):
    img_buffer = collections.deque(maxlen=5)
    last_img = None
    camera_matrix = None
    dist_coeffs = None
    trans = None

    def __init__(self):
        super().__init__('lidar_overlay')
        self.br = CvBridge()

        self.declare_parameter('t_x',0.0)
        self.declare_parameter('t_y',0.0)
        self.declare_parameter('t_z',0.0)

        self.declare_parameter('r_x',0.0)
        self.declare_parameter('r_y',0.0)
        self.declare_parameter('r_z',0.0)

        self.declare_parameter('camera_frame_id', "camera_link")
        self.declare_parameter('lidar_frame_id', "velodyne_link")


        qos = rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.VOLATILE, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=1, history=rclpy.qos.HistoryPolicy.KEEP_LAST)
        self.subImage = self.create_subscription(CompressedImage, 'image_bb/compressed', self.onImage, qos)
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, qos)

        self.subLidar = self.create_subscription(PointCloud2, 'velodyne_points', self.onLidar, 10)
        self.pub = self.create_publisher(CompressedImage, 'lidar_overlay/compressed', qos)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)

    def updateTransform(self, src_frame: str, trg_frame: str):
        try:
            tr = self.tf_buffer.lookup_transform(src_frame,
                    trg_frame, rclpy.time.Time()).transform
            rot = euler_from_quaternion([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
            self.trans = (
                np.array([tr.translation.x, tr.translation.y, tr.translation.z],float),
                np.array([rot[0], rot[1], rot[2]]                              ,float)
            )
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            print(f'Unable to find the transformation from {src_frame} to {trg_frame}')
            return False

    def onImage(self, msg):
        self.img_buffer.append(msg)

    #returns the closest image in time to the time stamp from the img_buffer
    def findClosestImage(self, stamp):
        closest = None
        min_diff = 1e9
        for img in self.img_buffer:
            diff = abs(img.header.stamp.nanosec - stamp.nanosec)
            if diff < min_diff:
                closest = img
                min_diff = diff
        return closest
    
    def onLidar(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        
        if self.trans is None:
            if not self.updateTransform(self.get_parameter('camera_frame_id').get_parameter_value().string_value, 
                                        self.get_parameter('lidar_frame_id').get_parameter_value().string_value):
                return
            
        img = self.findClosestImage(msg.header.stamp)
        if img is None:
            return
        print("Time Diff",(msg.header.stamp.nanosec - img.header.stamp.nanosec)*1e-6, "ms")
        img = self.br.compressed_imgmsg_to_cv2(img)

        tvec = self.trans[0] + np.array([self.get_parameter('t_x').get_parameter_value().double_value,
                                         self.get_parameter('t_y').get_parameter_value().double_value,
                                         self.get_parameter('t_z').get_parameter_value().double_value], float)
        rvec = self.trans[1] + np.array([self.get_parameter('r_x').get_parameter_value().double_value,
                                        self.get_parameter('r_y').get_parameter_value().double_value,
                                        self.get_parameter('r_z').get_parameter_value().double_value], float)

        print("trans", tvec, rvec)            
        t = time.time()

        lidar = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity"))        

        lidar_new = np.zeros((lidar.shape[0], 3))
        lidar_new[:,0] = lidar["y"]
        lidar_new[:,1] = lidar["z"]
        lidar_new[:,2] = -lidar["x"]
    
        # TODO not sure about the rvec order
        projPoints,_ = cv2.projectPoints(lidar_new, np.array([rvec[2], rvec[0], rvec[1]]), np.array([tvec[1], tvec[2], -tvec[0]]), self.camera_matrix, self.dist_coeffs)
        projPoints = projPoints.reshape(-1,2)

        t2 = time.time()
        #i_max = lidar["intensity"].max()
        #i_min = lidar["intensity"].min()
        i_min = 0
        i_max = 300
        for idx, point in enumerate(projPoints):
            if point[0] >= 640 or point[1] >= 480 or point[0] < 0 or point[1] < 0:
                continue
            cv2.circle(img, (int(point[0]), int(point[1])), 1, color(lidar["intensity"][idx] ,i_min,i_max), -1)

        print(f'Total in ({(1E3 * (time.time() - t)):.1f}ms) computation in ({(1E3 * (t2 - t)):.1f}ms)')
        
        self.pub.publish(self.br.cv2_to_compressed_imgmsg(img))


#returns a color with hue corresponding to value, between v_min and v_max
def color(value, v_min, v_max):
        hue = min(360 * (value - v_min) / (v_max - v_min), 360)
        return cv2.cvtColor(np.uint8([[[int(hue),255,255]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()

def main(args=None):
    rclpy.init(args=args)
    my_node = LidarOverlay()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()