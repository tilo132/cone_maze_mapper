import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType

import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf_transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_point

from cat_msgs_ros.msg import PolygonsStamped
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
    CameraInfo,
    LaserScan
)
from geometry_msgs.msg import PointStamped
import math
import struct
from message_filters import ApproximateTimeSynchronizer, Subscriber

class LidarBbox(Node):
    QoS = rclpy.qos.QoSProfile(
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        depth=1,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST
    )
    ReliableQoS = rclpy.qos.QoSProfile(
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        depth=1,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST
    )

    camera_matrix = None
    dist_coeffs = None
    trans = None
    ques = {}

    def __init__(self):
        super().__init__('lidar_bbox')
        
        #region Parameters
        self.declare_parameter(
            'frame_camera',
            'camera_link',
            ParameterDescriptor(
                name='frame_camera',
                description='The frame of the camera',
                type=ParameterType.PARAMETER_STRING,
            )
        )
        self.declare_parameter(
            'frame_lidar',
            'velodyne_link',
            ParameterDescriptor(
                name='frame_lidar',
                description='The frame of the lidar',
                type=ParameterType.PARAMETER_STRING,
            )
        )
        self.declare_parameter(
            'frame_base',
            'base_link',
            ParameterDescriptor(
                name='frame_base',
                description='The frame of the robot base',
                type=ParameterType.PARAMETER_STRING,
            )
        )        
        self.declare_parameter(
            'publish_pointcloud',
            True,
            ParameterDescriptor(
                name='publish_pointcloud',
                description='Whether to publish the pointcloud',
                type=ParameterType.PARAMETER_BOOL,
            )
        )
        self.declare_parameter(
            'publish_laserscan',
            True,
            ParameterDescriptor(
                name='publish_laserscan',
                description='Whether to publish the laserscan',
                type=ParameterType.PARAMETER_BOOL,
            )
        )
        self.declare_parameter(
            'publish_transformed_pointcloud',
            True,
            ParameterDescriptor(
                name='publish_transformed_pointcloud',
                description='Whether to publish the transformed pointcloud',
                type=ParameterType.PARAMETER_BOOL,
            )
        )
        self.declare_parameter(
            'publish_estimated_cones',
            True,
            ParameterDescriptor(
                name='publish_estimated_cones',
                description='Whether to publish the estimated cone positions as a pointcloud',
                type=ParameterType.PARAMETER_BOOL,
            )
        )
        self.declare_parameter(
            'publish_filtered_pointcloud',
            True,
            ParameterDescriptor(
                name='publish_topic_filtered',
                description='Whether to publish filtered pointcloud',
                type=ParameterType.PARAMETER_BOOL,
            ))
        self.declare_parameter(
            'time_synchronizer_buffer_size',
            20,
            ParameterDescriptor(
                name='time_synchronizer_buffer_size',
                description='',
                type=ParameterType.PARAMETER_INTEGER,
            ))
        self.declare_parameter(
            'time_synchronizer_max_delay',
            0.2,
            ParameterDescriptor(
                name='time_synchronizer_max_delay',
                description='',
                type=ParameterType.PARAMETER_DOUBLE,
            ))
        self.declare_parameter(
            'laserscan_height_threshold',
            0.035,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
            ))
        
        (
            self.FRAME_LIDAR,
            self.FRAME_CAMERA,
            self.FRAME_BASE,
            self.TIME_SYNCHRONIZER_BUFFER_SIZE,
            self.TIME_SYNCHRONIZER_MAX_DELAY,
        ) = [p.value for p in self.get_parameters([
            'frame_lidar',
            'frame_camera',
            'frame_base',
            'time_synchronizer_buffer_size',
            'time_synchronizer_max_delay',
        ])]

        #endregion

        #region Subscribers
        tss = ApproximateTimeSynchronizer(
            [Subscriber(self, PolygonsStamped, "bbox", qos_profile=self.ReliableQoS),
             Subscriber(self, PointCloud2, "velodyne_points", qos_profile=self.QoS)],
            self.TIME_SYNCHRONIZER_BUFFER_SIZE, self.TIME_SYNCHRONIZER_MAX_DELAY)
        tss.registerCallback(self.onData)

        self.create_subscription(CameraInfo, "camera_info", self.camera_info_callback, self.QoS)
        #endregion

        #region Publishers
        self.pub = {
            "pointcloud": self.create_publisher(PointCloud2, "cones/pointcloud", self.QoS),
            "laserscan": self.create_publisher(LaserScan, "cones/laserscan", self.QoS),
            "filtered": self.create_publisher(PointCloud2, "velodyne_points/filtered", self.QoS),
            "transformed": self.create_publisher(PointCloud2, "velodyne_points/transformed", self.QoS),
            "estimated_cones": self.create_publisher(PointCloud2, "cones/estimated", self.QoS),
        }
        #endregion

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)

    def updateTransform(self):
        try:
            self.lidar_camera_trans = self.tf_buffer.lookup_transform(self.FRAME_CAMERA, self.FRAME_LIDAR, rclpy.time.Time())
            self.camera_base_trans = self.tf_buffer.lookup_transform(self.FRAME_CAMERA, self.FRAME_BASE, rclpy.time.Time())
            
            self.lidar_base_trans = self.tf_buffer.lookup_transform(self.FRAME_BASE, self.FRAME_LIDAR, rclpy.time.Time())

            #create rotation matrix from quaternion
            rot = euler_from_quaternion([
                self.lidar_camera_trans.transform.rotation.x,
                self.lidar_camera_trans.transform.rotation.y,
                self.lidar_camera_trans.transform.rotation.z,
                self.lidar_camera_trans.transform.rotation.w
            ])

            #remapp the translation vector to the cv2 coordinate system
            self.trans = (
                np.array([
                    -self.lidar_camera_trans.transform.translation.y,
                    -self.lidar_camera_trans.transform.translation.z,
                    self.lidar_camera_trans.transform.translation.x,
                ], float),
                np.array([-rot[2],-rot[0],rot[1]], float)
            )
            return True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            print(f'Unable to find the transformation')
            return False

    def pnpBB(self, bbox):
        '''
        perform a Perspective-n-Point (PnP) pose estimation based on a 2D bounding box 
        of an object detected in an image. The objective is to estimate the 3D position
         of the object in the world coordinates using the camera's intrinsic matrix and 
         distortion coefficients.
        '''
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None

        x: int = bbox[0].x
        y: int = bbox[0].y
        width: int = bbox[1].x - bbox[0].x
        height: int = bbox[3].y - bbox[0].y

        points = np.array([
            [x, y],
            [x + width, y],
            [x + width, y + height],
            [x, y + height]
        ])

        #bounding box points of a cone in the camera frame
        object_points = np.array([[-0.0375,-0.04,0], 
                                  [0.0375,-0.04,0], 
                                  [0.0375, 0.04,0], 
                                  [-0.0375, 0.04,0]])
        _, rvec, tvec = cv2.solvePnP(object_points, points, self.camera_matrix, self.dist_coeffs)
        if tvec is None:
            return None
        
        #remapp the cv2 point to a ros point and transform it to the base frame
        ps = PointStamped()
        ps.header.frame_id = self.FRAME_BASE
        ps.point.x = float(tvec[2])
        ps.point.y = float(-tvec[0])
        ps.point.z = float(-tvec[1])
        ps = do_transform_point(ps, self.camera_base_trans)
        
        p = [ps.point.x, ps.point.y, ps.point.z]

        # normalize tvec we expect all the cones to be on the ground plane, so we set z to 0.04 (half cone height)
        dist = np.linalg.norm(p)
        p[2] = 0
        p = p / np.linalg.norm(p) * dist
        p[2] = 0.04
        return p
   
    def onData(self, bb_msg, lidar_msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        
        if self.trans is None:
            if not self.updateTransform():
                return
                
        print("Time Diff",(lidar_msg.header.stamp.nanosec - bb_msg.header.stamp.nanosec)*1e-6, "ms")        

        t = time.time()
        lidar = pc2.read_points(lidar_msg, skip_nans=True)
        
        # Remapp lidar points to the cv2 coordinate system
        lidar_points = np.zeros((lidar.shape[0], 3))
        lidar_points[:,0] = -lidar["y"]
        lidar_points[:,1] = -lidar["z"]
        lidar_points[:,2] = lidar["x"]

        # Projects the transformed lidar points onto the camera's image plane using the cv2.projectPoints
        # function. The projPoints array will contain the projected 2D points on the image plane.        
        projPoints, _ = cv2.projectPoints(lidar_points, self.trans[1], self.trans[0], self.camera_matrix, self.dist_coeffs)
        projPoints = projPoints.reshape(-1,2)
        t2 = time.time()


        #transform the lidar pointcloud to the base frame
        lidar_points[:,0] = lidar["x"]
        lidar_points[:,1] = lidar["y"]
        lidar_points[:,2] = lidar["z"]
        translation = np.array([self.lidar_base_trans.transform.translation.x, self.lidar_base_trans.transform.translation.y, self.lidar_base_trans.transform.translation.z])

        #create rotation matrix from quaternion
        rot,_ = cv2.Rodrigues(euler_from_quaternion([
            self.lidar_base_trans.transform.rotation.x,
            self.lidar_base_trans.transform.rotation.y,
            self.lidar_base_trans.transform.rotation.z,
            self.lidar_base_trans.transform.rotation.w]))
        #transformation matrix * pointcloud + offset
        transformed = (rot @ lidar_points.T).T + translation
        
        #filter out the points of the floor by a basic z filter
        # filer_idxs = transformed[:,2] > self.get_parameters(['laserscan_height_threshold'])[0].value
        # projPoints = projPoints[filer_idxs]
        # transformed = transformed[filer_idxs]
        # lidar = lidar[filer_idxs]

        lidar_filter = np.zeros(transformed.shape[0], dtype=bool)

        cones_out = []
        for polygon in bb_msg.polygons:
            polygon_id: int = polygon.id
            polygon_color: int = polygon.color
            polygon: lidar_msg.msg.Polygon = polygon.polygon
            #position cones based on the bounding box
            estimated: list = self.pnpBB(polygon.points)

            PADDING = 10
            top_left =      np.array([polygon.points[0].x-PADDING, polygon.points[0].y-PADDING])
            bottom_right =  np.array([polygon.points[2].x+PADDING, polygon.points[2].y+PADDING])       

            #index of all points in the bounding box that are also within a given distance to the estimated cone position
            inbb_idx = np.logical_and(
                np.all(
                    np.logical_and(
                        top_left <= projPoints,
                        projPoints <= bottom_right
                    ), 
                    axis=1
                )
                ,(np.linalg.norm(np.array(transformed)-np.array(estimated), axis=1) <= 0.4) if estimated is not None else True
            )
            
            if self.get_parameter('publish_filtered_pointcloud').value:
            #    export filtered points on 'velodyne_points/filtered' for visual debugging
                lidar_filter = np.logical_or(lidar_filter, inbb_idx)
                
            inbb = transformed[inbb_idx]
            if inbb.shape[0] == 0:#no points in bounding box
                continue
            
            #Find the center of the cluster
            const_weights = np.exp(-10*np.linalg.norm(inbb-estimated, axis=1)) * lidar["intensity"][inbb_idx]
            def kernel(x: np.ndarray):
                return np.exp(10*x[:,2]) * np.exp(0.1*(-x[:,0]+max(x[:,0]))) * const_weights
            cluster_center = slide(inbb, estimated, kernel)

            cones_out.append([polygon_id, polygon_color, cluster_center])
               
        t3 = time.time()
        print(f'Found {len(cones_out)} cones')

        
        #publish estimated cones
        if self.get_parameter('publish_estimated_cones').value:
            pnp_cones_out = [[polygon.id, polygon.color,self.pnpBB(polygon.polygon.points)] for polygon in bb_msg.polygons]
            print(f'Found {len(pnp_cones_out)} cones with pnp')

            cones = points_to_pointcloud2(pnp_cones_out)
            cones.header.stamp = lidar_msg.header.stamp
            cones.header.frame_id = self.FRAME_BASE
            self.pub["estimated_cones"].publish(cones)

        #publish filtered pointcloud
        if self.get_parameter('publish_filtered_pointcloud').value:
            filtered_msg = pc2.create_cloud(lidar_msg.header, lidar_msg.fields, lidar[lidar_filter])
            filtered_msg.header.frame_id = self.FRAME_LIDAR
            filtered_msg.header.stamp = lidar_msg.header.stamp
            self.pub["filtered"].publish(filtered_msg)

        if self.get_parameter('publish_pointcloud').value:
            cloud = points_to_pointcloud2(cones_out)
            cloud.header.stamp = lidar_msg.header.stamp
            cloud.header.frame_id = self.FRAME_BASE
            self.pub["pointcloud"].publish(cloud)

        if self.get_parameter('publish_laserscan').value:
            scan = self.points_to_lasercan(cones_out, transformed)
            scan.header.stamp = lidar_msg.header.stamp
            scan.header.frame_id = self.FRAME_BASE
            self.pub["laserscan"].publish(scan)
        print(f'Total in ({(1E3 * (t3 - t)):.1f}ms) computation in ({(1E3 * (t3 - t2)):.1f}ms) Projecting in ({(1E3 * (t2 - t)):.1f}ms)')


    def points_to_lasercan(self, cones, points):
        scan_msg = LaserScan()

        scan_msg.angle_min = -90/360*math.pi*2  # Minimum angle of the laser scan
        scan_msg.angle_max = 90/360*math.pi*2  # Maximum angle of the laser scan
        scan_msg.angle_increment = 0.01  # Angular distance between measurements

        scan_msg.time_increment = 0.0  # Time between measurements (if sensors are moving)
        scan_msg.scan_time = 0.0  # Time taken to complete a full scan

        scan_msg.range_min = 0.0  # Minimum range value
        scan_msg.range_max = 10.0  # Maximum range value

        scan_len = (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment
        # List to store the range measurements initialized with inf values
        scan_msg.ranges = [float('inf')]*(math.ceil(scan_len))
        
        t = time.time()
        points = points[points[:,2] > self.get_parameters(['laserscan_height_threshold'])[0].value]
        #Index of the points of the pointcloud
        indexes = np.round((np.arctan2(points[:,1], points[:,0]) - scan_msg.angle_min) / scan_msg.angle_increment).astype(int)
        
        #Stay in the angle range of the laserscan
        goodidx = np.logical_and(indexes >= 0, indexes < len(scan_msg.ranges))
        indexes = indexes[goodidx]
        points = points[goodidx]

        #Calculate the range of the points
        r = np.hypot(points[:,0], points[:,1])
        ranges = np.array(scan_msg.ranges)        
        
        #Handle duplicate indexes, only keep the one with the smallest range
        duplicate_indexes, unique_indices = np.unique(indexes, return_inverse=True)
        # Create an array to store the minimum values for each unique index
        min_values = np.full(duplicate_indexes.shape, np.inf)
        # Use np.minimum.at to calculate the minimum values for each unique index
        np.minimum.at(min_values, unique_indices, r)
        # Update the values in the arr array based on the minimum values
        np.put(ranges, duplicate_indexes, min_values)

        scan_msg.ranges = list(ranges.astype(float))
        t2=time.time()
        
        cone_radius = 0.0305
        #Transform cones to base_link
        if len(cones) == 0:
            return scan_msg


        for _id, _color, (x, y, _z) in cones:
            # Calculate the range from the origin to the point
            initial_angle = math.atan2(y, x) - scan_msg.angle_min
            initial_distance = np.hypot(x, y)
            initial_idx = np.clip(round(initial_angle / scan_msg.angle_increment), 0, len(scan_msg.ranges)-1)
     
            if initial_distance < scan_msg.ranges[initial_idx]:
                scan_msg.ranges[initial_idx] = initial_distance - cone_radius
            
            for dir in range(2):
                i=1
                while True:
                    angle_diff = i*scan_msg.angle_increment * (1 if dir==1 else -1)
                    radial_distance = 2*initial_distance*math.tan(angle_diff/2)
                    if abs(radial_distance) < cone_radius:
                        index = round(((initial_angle + angle_diff) / scan_msg.angle_increment) % scan_len)
                        distance = initial_distance - (cone_radius**2 - radial_distance**2)**(0.5)
                        if distance < scan_msg.ranges[index]:
                            scan_msg.ranges[index] = distance
                    else:
                        break
                    i+=1

        print(f'Points to laserscan in {(1E3 * (t2 - t)):.1f}ms, cones in {(1E3 * (time.time() - t2)):.1f}ms')

        return scan_msg


def points_to_pointcloud2(points, only_points = False):
    # Create PointCloud2 message
    pointcloud2_msg = PointCloud2()

    # Convert fields to PointField format
    pointcloud2_msg.fields = [PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'id',offset = 12,datatype = PointField.INT32,   count = 1),
                              PointField(name='color',offset = 16,datatype = PointField.INT32,   count = 1),
                              ]
    pointcloud2_msg.is_bigendian = False
    pointcloud2_msg.point_step = 20
    pointcloud2_msg.row_step = 20 * len(points)
    pointcloud2_msg.height = 1
    pointcloud2_msg.width = len(points)

    # Convert points to bytes
    byte_points = []
    if only_points:
        for p in points:
            byte_points.append(struct.pack('fffii', p[0], p[1],p[2], 0, 0))
    else:
        for id,color, (x,y,z) in points:
            byte_points.append(struct.pack('fffii', x, y, z, int(id), int(color)))
    pointcloud2_msg.data = b''.join(byte_points)
    return pointcloud2_msg

def slide(points: np.ndarray, seed: np.ndarray, kernel, eps=0.01, max_iter=10):
    for i in range(max_iter):
        # Calculate weights using the provided kernel function
        weights = kernel(points - seed)
        # Shift weights to ensure they are non-negativ
        weights -= min(weights)
        if np.sum(weights) == 0:
            break
        # Calculate the weighted average of points to determine the update direction
        delta = np.average(points, weights=weights, axis=0)-seed
        #check for not a number  or inf
        if any(np.isnan(delta)) or any(np.isinf(delta)):
            break
        seed += delta
        if np.linalg.norm(delta) < eps:
            break
    return seed

def main(args=None):
    rclpy.init(args=args)
    my_node = LidarBbox()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
