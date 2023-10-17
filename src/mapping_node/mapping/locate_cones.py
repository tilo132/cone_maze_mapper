import collections
import time
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import math

import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf_transformations import euler_from_quaternion

from cat_msgs_ros.msg import (
    Cones, Cone, Poi
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
)
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)
import struct
from collections import deque

class ConeCluster:
    """A class that holds the datapoints we consider to be describing the same cone."""
    
    def __init__(self, color, id, max_len, init_point, init_covar, visited):
        self.color = color
        self.id = id
        self.max_len = max_len
        self.confidence = 0.0
        self.time_stamp = 0
        self.visited = visited

        self.cluster_center = np.array(init_point)        
        self.data = deque(maxlen=max_len)
        self.confidences = deque(maxlen=max_len)

        self.add(init_point, init_covar, visited)

    def add(self, point: list[int], covariance, visited):
        """Adds a new point to the cluster and recalculates the center and confidence"""

        # print("visited ",self.visited, visited)
        self.visited = self.visited or visited
        self.time_stamp = 0
        self.data.append(point)
        self.confidences.append(1/covariance)
        self.calculate_center()

        self.calculate_confidence()

    def calculate_confidence(self):
        self.confidence = np.array(self.confidences)[:,2].sum()

    def to_numpy(self):
        return np.array(self.data)

    def merge(self, other):
        self.time_stamp = 0

        # TODO extend should take into account the que length
        self.data.extend(other.data)
        self.confidences.extend(other.confidences)
        
        #recalculate
        self.calculate_center()
        self.calculate_confidence()

    def calculate_center(self):
        """Returns the center of the cluster as a numpy array"""

        np_data = self.to_numpy()
        confs = np.array(self.confidences)

        # Weighted Average by normalized covariance
        # X
        kernel = confs[:,0]
        kernel = np.flip(kernel / kernel.sum())
        self.cluster_center[0] = np.convolve(np_data[:,0], kernel, 'valid')
        # Y
        kernel = confs[:,1]
        kernel = np.flip(kernel / kernel.sum())
        self.cluster_center[1] = np.convolve(np_data[:,1], kernel, 'valid')
    
        #Z: There is no confidence for z so we just use a moving average
        width = len(np_data)
        kernel = np.ones(width) / width
        self.cluster_center[2] = np.convolve(np_data[:,2], kernel, 'valid')

class ConeClusterCollection:
    """A class that holds all the cone clusters"""

    def __init__(self):
        self.color_to_ids: dict[int, list[int]] = {
            0: [], #Blue
            1: [], #Green
            2: [], #Pink
            3: [], #Red
            4: [], #Yellow
            5: [], #POI (not a cone)
        }
        self.id_to_cluster: dict[int, ConeCluster] = {}
        # self.cone_id_to_mapping_id = {}

    def add(self, color, id, point: list[int], covariance, max_rematch_distance, visited) -> int:
        """
        Adds a new cone to the cluster.
        If the id is not already known, the id will either be matched to an existing cone or a new id will be created.

        Args:
            color (int): The color of the cone
            id (int): The id of the cone from the incoming message
            point (list[int]): The position of the cone as in [x,y,z]
            covariance (list[int]): The covariance of the cone as in [covar_x, covar_y, covar_yaw, covar_average]
            max_rematch_distance (float): The maximum distance between two cones to be considered the same cone
        Returns:
            int: The remapped id of the cone in the cluster
        """


        # if id in self.cone_id_to_mapping_id:
        #     # cones come with id from the camera, so if we have seen this id before, put it in the same cluster
        #     remapped_id = self.cone_id_to_mapping_id[id]
        #     self.id_to_cluster[remapped_id].add(point, covariance)
        #     return remapped_id
        
        min, min_clust = float("inf"), None
        #find closest cone of same color
        for cluster in self.get_by_color(color):
            dis = np.linalg.norm(cluster.cluster_center - np.array(point))
            if dis < min:
                min, min_clust = dis, cluster

        if min < max_rematch_distance:
            # found cone that is close enough, remap id
            min_clust.add(point, covariance, visited)
            # self.cone_id_to_mapping_id[id] = min_clust.id
            return min_clust.id
        
        # no close enough cone found, create new cluster entry
        remapped_id = self._free_id()
        self.id_to_cluster[remapped_id] = ConeCluster(color, remapped_id, 100, point, covariance, visited)
        # self.id_to_cluster[remapped_id].add(point, covariance)
        self.color_to_ids[color].append(remapped_id)
        # self.cone_id_to_mapping_id[id] = remapped_id
        return remapped_id
    
    def _free_id(self) -> int:
        '''returns the next free id'''
        id = 0
        while id in self.id_to_cluster:
            id += 1
        return id

    def get_by_color(self, color: int) -> list[ConeCluster]:
        '''returns all histories of a given color'''
        return [self.id_to_cluster[id] for id in self.color_to_ids[color]]
    
    def increment_time_stamp(self):
        '''increments the time stamp of all histories'''

        for hist in self.id_to_cluster.values():
            hist.time_stamp += 1

    def get_all_cones(self) -> list[ConeCluster]:
        '''returns all cones'''

        #All but color == 5 (the Pois)
        return np.hstack(
            [self.get_by_color(color) for color in range(5)]
        )

    def merge(self, idx_pairs):
        '''merges Histories takes an array of index pairs, index of id_to_cluster dict'''
        to_del = []
        for [idx1, idx2] in idx_pairs:
            keys = list(self.id_to_cluster.keys())
            id1 = keys[idx1]
            id2 = keys[idx2]

            hist1 = self.id_to_cluster[id1]
            hist2 = self.id_to_cluster[id2]
            if hist1.color != hist2.color:
                return
            print("merging", hist1.cluster_center, hist2.cluster_center, hist1.color)
                
            hist1.merge(hist2)
    #        for key,val in self.cone_id_to_mapping_id:
            #Merge cone_id_to_mapping_id
            to_del.append(id2)

        self.delete(to_del)

    def delete(self, remapped_ids):
        '''delete Histories by remapped id takes an array of remapped ids'''
        remapped_ids.sort()
        remapped_ids.reverse()
        for remapped_id in remapped_ids:
            #Remove Remapped id from all data structures
            # self.cone_id_to_mapping_id = {key:val for key, val in self.cone_id_to_mapping_id.items() if val != remapped_id}

            for l in self.color_to_ids.values():
                if remapped_id in l:
                    l.remove(remapped_id)

            del self.id_to_cluster[remapped_id]#Error: KeyError: 11

class LocateCones(Node):
    QOS = rclpy.qos.QoSProfile(
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        depth=1,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST
    )

    FRAME_MAP = "map"
    FRAME_ODOM = "odom"

    last_odom_map = None
    base_odom = None
    odom_map = None

    cone_histories: ConeClusterCollection = ConeClusterCollection()    

    poses = collections.deque(maxlen=10)
    
    def __init__(self):
        super().__init__('locate_cones')
        print("Init")
        
        self.create_subscription(PointCloud2, "/cones/pointcloud", self.onCones, self.QOS)
        self.create_subscription(Poi, "/poi", self.onPoi, self.QOS)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.onPose, self.QOS)

        self.pub = {
            "cones_pointcloud": self.create_publisher(PointCloud2, "cones/map", self.QOS),
            "pois_pointcloud": self.create_publisher(PointCloud2, "poi/map", self.QOS),
            "cones": self.create_publisher(Cones, "cones/cones", self.QOS),
        }
        
        self.declare_parameter('max_rematch_distance', 0.25)
        print("max_rematch_distance", self.get_parameter('max_rematch_distance').value)

        #Age a cluster needs to be old to be considered for cleanup
        #This needs to be high enought that clusters that are just being created are not deleted right away
        self.declare_parameter('cleanup_time_stamp_threshold', 10)
        # How much lower the confidence of a cluster has to be compared to the average confidence of clusters in the comparison radius to be cleaned up
        self.declare_parameter('cleanup_confidence_scale', 0.5)
        # Radius in meters in which clusters are compared to each other
        self.declare_parameter('cleanup_comparison_radius', 1.0)
        
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=10))
        tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)

    def getTransform(self, cones_msg):
        '''returns the transformation from camera frame to map frame'''
        try:
            self.base_odom = self.tf_buffer.lookup_transform(self.FRAME_ODOM, cones_msg.header.frame_id, cones_msg.header.stamp, rclpy.duration.Duration(seconds=1))
            self.odom_map = self.tf_buffer.lookup_transform(self.FRAME_MAP, self.FRAME_ODOM, rclpy.time.Time(), rclpy.duration.Duration(seconds=1))
            
            # if self.last_map_odom is not None:
            #     dist = [map_odom.transform.translation.x - self.last_map_odom.transform.translation.x, map_odom.transform.translation.y - self.last_map_odom.transform.translation.y, map_odom.transform.translation.z - self.last_map_odom.transform.translation.z]

            #     rot_z1 = euler_from_quaternion([map_odom.transform.rotation.x,map_odom.transform.rotation.y,map_odom.transform.rotation.z,map_odom.transform.rotation.w])[2]
            #     rot_z2 = euler_from_quaternion([self.last_map_odom.transform.rotation.x,self.last_map_odom.transform.rotation.y,self.last_map_odom.transform.rotation.z,self.last_map_odom.transform.rotation.w])[2]

            #     # if the difference between the two odoms is too big, we assume that the robot has been teleported (slam has reset the position)
            #     angle = abs(rot_z1 - rot_z2) % math.pi
            #     if dist[0] > 0.15 or dist[1] > 0.15 or angle > 0.15:
            #         t = time.time()
            #         self.get_logger().info("map_odom moved too much, adjusting cone confidence")
            #         for hist in self.cone_histories.id_to_cluster.values():
            #             for confidence in hist.confidences:
            #                 confidence[0] = confidence[0] / (1+dist[0] + angle*2)#2 can be adjusted to change the impact of the angle on the confidence 2: 11° ~> 0.4m
            #                 confidence[1] = confidence[1] / (1+dist[1] + angle*2)
            #             hist.calculate_confidence()
            #         self.get_logger().info("adjusting cone confidence took "+ str(time.time()-t) +"seconds")

            # self.last_odom_map = self.odom_map
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            pass
        
        print(f'Timeout: Unable to find the transformation . ')
        return False
    
    def transform_cones(self, cones):
        '''transforms cones from camera frame to map frame'''
        translation_odom = np.array([self.base_odom.transform.translation.x, self.base_odom.transform.translation.y, self.base_odom.transform.translation.z])
        rot_odom,_ = cv2.Rodrigues(euler_from_quaternion([self.base_odom.transform.rotation.x, self.base_odom.transform.rotation.y, self.base_odom.transform.rotation.z, self.base_odom.transform.rotation.w]))
        translation_map = np.array([self.odom_map.transform.translation.x, self.odom_map.transform.translation.y, self.odom_map.transform.translation.z])
        rot_map,_ = cv2.Rodrigues(euler_from_quaternion([self.odom_map.transform.rotation.x, self.odom_map.transform.rotation.y, self.odom_map.transform.rotation.z, self.odom_map.transform.rotation.w]))
        
        # transform the x,y,z coordinates of the cones
        cones_odom = (rot_odom @ cones[:,0:3].T).T + translation_odom
        cones_map = (rot_map @ cones_odom.T).T + translation_map

        # add the addidtional information back to the cones
        cones_map = np.hstack((cones_map, cones[:,3:10]))
        
        return cones_map
    
    def readPoi(self, poi_msg):
        '''reads poi from poi_msg'''
        return np.array([[
            poi_msg.pose.pose.position.x, poi_msg.pose.pose.position.y, poi_msg.pose.pose.position.z,
            poi_msg.id,#id
            5,#color 5 is for poi's
        ]])

    def readCones(self, cones_msg):
        '''reads cones from cones_msg'''
        cones = pc2.read_points(cones_msg, skip_nans=True)
        cones_ary = np.zeros((cones.shape[0], 5))
        cones_ary[:,0] = cones["x"]
        cones_ary[:,1] = cones["y"]
        cones_ary[:,2] = cones["z"]
        cones_ary[:,3] = cones["id"]
        cones_ary[:,4] = cones["color"]
        return cones_ary
    
    def calculate_cone_covariance(self, pose_covariance, cones):
        # adjust covariance based on distance by: covar = covar * (a*dist^b+1)
        # this is done to give cones that are further away a higher covariance
        # because we found that this gives better results
        a = 1
        b = 2
        dists = np.linalg.norm(cones[:,0:3], axis=1)
        visited = dists <= 2.5
        c = a*dists**b +1
        
        covariance = dists[:, np.newaxis] * pose_covariance.reshape(1, len(pose_covariance))
        return np.hstack([cones, covariance, visited[:,np.newaxis]])
    
    def cluster_level_computation(self, max_rematch_distance, cleanup_time_stamp_threshold, cleanup_comparison_radius, cleanup_confidence_scale):
        """Merge cluster that are close and cleanup old/bad clusters"""
        clusters = self.cone_histories.id_to_cluster
        if len(clusters) == 0:
            return
        
        points = np.array([[a.cluster_center[0], a.cluster_center[1]] for a in list(clusters.values())])
        # an array per point of differences between that point and all points
        differences = points[:, None] - points
        distances = np.linalg.norm(differences, axis=2)
        upper_triangle_mask = np.triu(np.ones_like(distances), k=1) == 1

        # Find the indices of points that have distances smaller than the threshold
        row_indices, col_indices = np.where(np.bitwise_and(distances < max_rematch_distance, upper_triangle_mask))

        # Combine row and column indices to get pairs of points
        pairs = list(zip(row_indices, col_indices))

        #Merge clusters
        self.cone_histories.merge(pairs)
        

        if len(pairs) > 0:
            # TODO: We have to recalc the distances because in case of a merge they are not up to date but this is not efficient
            points = np.array([[a.cluster_center[0], a.cluster_center[1]] for a in list(clusters.values())])
            # an array per point of differences between that point and all points
            differences = points[:, None] - points
            distances = np.linalg.norm(differences, axis=2)
            clusters = self.cone_histories.id_to_cluster

        confidences = np.array([cl.confidence for cl in list(clusters.values())])
        ages = np.array([cl.time_stamp for cl in list(clusters.values())])
        
        to_del = []

        #Compare each cluster to all clusters in the comparison radius and delete it if its confidence is too low
        for row in range(distances.shape[0]):
            id = list(clusters.keys())[row]

            # if cluster is old enough
            if clusters[id].time_stamp > cleanup_time_stamp_threshold:
                mask = np.ones(distances.shape[0], dtype=bool)
                mask[row] = False
                # average confidences of clusters in comparison radius
                radius_filter = np.bitwise_and(distances[row] < cleanup_comparison_radius, mask)

                if len(radius_filter) == 0 or radius_filter.sum() == 0:
                    continue
                avg_conf = confidences[radius_filter].mean()
                avg_age = ages[radius_filter].mean()
                
                # if cluster is not confident enough delete it
                old = ages[row] * cleanup_confidence_scale / 4 > avg_age
                if (confidences[row] < avg_conf * cleanup_confidence_scale) and clusters[id].color != 5:
                    self.get_logger().info("deleting "+ str(id) +" " + str(confidences[row]) +" " +str(avg_conf)+", old="+str(old))
                    to_del.append(id)


        self.cone_histories.delete(to_del)

    def onPoi(self, poi_msg):
        """Callback for poi message. Pois are handled like cones but with a color of 5"""
        pose_covariance = self.get_covariance(poi_msg.pose.header.stamp)
        
        self.getTransform(poi_msg.pose)
        if self.odom_map is None or self.base_odom is None:
            self.get_logger().info(f"no transform {self.odom_map} {self.base_odom} a")
            return
        
        data = self.readPoi(poi_msg)
        self.cone_computation(data, pose_covariance)
        
        # get all pois
        pois = self.cone_histories.get_by_color(5)

        new_cones_msg = pois_to_pointcloud2({
            hist.id: (hist.color, hist.cluster_center, hist.confidence) for hist in pois
        })
        new_cones_msg.header.frame_id = self.FRAME_MAP
        new_cones_msg.header.stamp = poi_msg.pose.header.stamp
        self.pub["pois_pointcloud"].publish(new_cones_msg)
        
        pass

    def onCones(self, cones_msg):
        '''Callback for cones message. Transforms cones to map frame and adds them to cone_histories and publish the cones ordered by color in an array of cones
            '''        
        pose_covariance = self.get_covariance(cones_msg.header.stamp)
        
        self.getTransform(cones_msg)
        if self.odom_map is None or self.base_odom is None:
            self.get_logger().info(f"no transform {self.odom_map} {self.base_odom} b")
            return
        
        cones = self.readCones(cones_msg)
        self.cone_computation(cones, pose_covariance)

        #all_cones that are older then cleanup_time_stamp_threshold +1
        all_cones = [[hist.id, hist.color, hist.confidence, hist.cluster_center] for hist in self.cone_histories.get_all_cones() if len(hist.data) >= 5 and hist.visited]
        new_cones_msg = Cones()
        new_cones_msg.header.frame_id = self.FRAME_MAP
        new_cones_msg.header.stamp = cones_msg.header.stamp
        
        for id, color, confidence, (x,y,z) in all_cones:
            new_cones_msg.cones.append(Cone(
                id=id,
                color=color,
                pose=Pose(
                    position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ))
            )
        self.pub["cones"].publish(new_cones_msg)

        #publish cones as pointcloud
        new_cones_msg = cones_to_pointcloud2(all_cones)
        new_cones_msg.header.frame_id = self.FRAME_MAP
        new_cones_msg.header.stamp = cones_msg.header.stamp
        self.pub["cones_pointcloud"].publish(new_cones_msg)
        self.get_logger().debug('publishing cones') 


    def cone_computation(self, data, pose_covariance):
        """Computes the covarainces, transofrms the cones adds them to clusters then merges and cleans up the clusters"""
        #print("shape: ",data.shape)
        data = self.calculate_cone_covariance(pose_covariance, data)
        data = self.transform_cones(data)
        
        self.cone_histories.increment_time_stamp()

        max_rematch_distance = self.get_parameter('max_rematch_distance').value
        for (x, y, z, id, color, covar_x, covar_y, covar_yaw, covar_all, visited) in data:
            remapped_id = self.cone_histories.add(int(color), int(id), [x, y, z], np.array([covar_x, covar_y, covar_all]), max_rematch_distance, visited)

        cleanup_time_stamp_threshold = self.get_parameter('cleanup_time_stamp_threshold').value
        cleanup_comparison_radius = self.get_parameter('cleanup_comparison_radius').value
        cleanup_confidence_scale = self.get_parameter('cleanup_confidence_scale').value

        self.cluster_level_computation(max_rematch_distance, cleanup_time_stamp_threshold, cleanup_comparison_radius, cleanup_confidence_scale)

    def onPose(self, pose_msg):
        """Callback for pose message. Saves the pose and covariance for later use"""
        covariance = np.array([
            pose_msg.header.stamp.sec,#time stamp
            pose_msg.pose.covariance[0],#x
            pose_msg.pose.covariance[7],#y
            pose_msg.pose.covariance[35],#yaw
            (pose_msg.pose.covariance[0]+pose_msg.pose.covariance[7]+pose_msg.pose.covariance[35])/3,#average of the three values
        ])
        self.poses.append(covariance)

    def get_covariance(self, stamp):
        """Returns the closest covariance to the given timestamp"""
        if len(self.poses) == 0:
            #If no pose is available yet, return a covariance of 1
            #this will allow this node to give output before slam has returned its first pose
            return np.array([1.0, 1.0, 1.0, 1.0, 1.0])

        #Find idx of pose with closest timestamp
        idx = np.argmin(abs(np.array(self.poses)[:,0].astype(float) - stamp.sec))
        return self.poses[idx][1:]
        

def pois_to_pointcloud2(pois):
    # Create PointCloud2 message
    pointcloud2_msg = PointCloud2()

    # Convert fields to PointField format
    pointcloud2_msg.fields = [PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'id',offset = 12,datatype = PointField.INT32,   count = 1),
                              PointField(name = 'conf',offset = 16,datatype = PointField.INT32,   count = 1),
                              ]
    pointcloud2_msg.is_bigendian = False
    pointcloud2_msg.point_step = 20
    pointcloud2_msg.row_step = 20 * len(pois)
    pointcloud2_msg.height = 1
    pointcloud2_msg.width = len(pois)

    # Convert points to bytes
    byte_points = []
    for id, (color, (x,y,z), conf) in pois.items():
        byte_points.append(struct.pack('fffif', x, y, z, int(id), float(conf)))
    pointcloud2_msg.data = b''.join(byte_points)
    return pointcloud2_msg

def cones_to_pointcloud2(points):
    # Create PointCloud2 message
    pointcloud2_msg = PointCloud2()

    # Convert fields to PointField format
    pointcloud2_msg.fields = [PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
                              PointField(name = 'id',offset = 12,datatype = PointField.INT32,   count = 1),
                              PointField(name = 'color',offset = 16,datatype = PointField.INT32,   count = 1),
                              PointField(name = 'conf',offset = 20, datatype = PointField.FLOAT32,   count = 1),
                              ]
    pointcloud2_msg.is_bigendian = False
    pointcloud2_msg.point_step = 24
    pointcloud2_msg.row_step = 24 * len(points)
    pointcloud2_msg.height = 1
    pointcloud2_msg.width = len(points)

    # Convert points to bytes
    byte_points = []
    for id, color, conf, (x,y,z) in points:
        byte_points.append(struct.pack('fffiif', x, y, z, int(id), int(color), float(conf)))
    pointcloud2_msg.data = b''.join(byte_points)
    return pointcloud2_msg

def main(args=None):
    rclpy.init(args=args)
    my_node = LocateCones()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
