from typing import Any

from cat_msgs_ros.msg import PolygonsStamped, Polygon
from geometry_msgs.msg import Point32, Polygon as GeoPolygon
from ultralytics import YOLO

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType
)
import time

class ConeTrackingNode(Node):
    """
    Cone Tracking Node: Tracks cones in a video stream and publishes the results

    Name: 
        cone_tracking

    Publishers:
        bbox (cat_msgs_ros.msg.Polygons): Bounding boxes of the tracked objects
        bboxS (cat_msgs_ros.msg.PolygonsStamped): Bounding boxes of the tracked objects with a timestamp
        bbox_over (CompressedImage): Bounding boxes of the tracked objects overlayed on the image

    Subscribers:
        image_raw (CompressedImage): Image to run Yolo and RekNet on
    """

    # Quality of Service
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

    # BGR Colors. Match with the specification of LabelStudio and Yolo
    colorByClass = {
        0: (180,95,26),   # Blue
        1: (126,194,46),  # Green
        2: (158,163,255), # Pink
        3: (36,27,224),   # Red
        4: (92,228,248)   # Yellow
    }


    def __init__(self):
        """
        Constructor
        """

        super().__init__('cone_tracking')
        
        self.declare_parameter(
            'overlay_bbox',
            True,
            ParameterDescriptor(
                name='overlay_bbox',
                description='whether to overlay the bounding boxes on the image',
                type=ParameterType.PARAMETER_BOOL
            )
        )
        self.declare_parameter(
            'yolo_weights',
            "./data/weights/best.pt",
            ParameterDescriptor(
                name='yolo_weights',
                description='path to the weights file for the yolo bounding box detection model',
                type=ParameterType.PARAMETER_STRING
            )
        )
        self.declare_parameter(
            'yolo_conf_threshold',
            0.75,
            ParameterDescriptor(
                name='yolo_conf_threshold',
                description='confidence threshold for the yolo bounding box detection model',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        # CV-Bridge for image conversion
        self.cv_bridge = CvBridge()
    
        # Subscriber to Image topic
        self.subscription: Subscription = self.create_subscription(
            msg_type=CompressedImage,
            topic='image_raw/compressed',
            callback=self.onImage,
            qos_profile=self.QoS
        )

        self.publisher: dict[str, Publisher] = {
            "stamped": self.create_publisher(
                msg_type=PolygonsStamped,
                topic='bbox',
                qos_profile=self.ReliableQoS
            ),
            "bbox_over": self.create_publisher(
                msg_type=CompressedImage,
                topic='image_bb/compressed',
                qos_profile=self.QoS
            ),
        }
 
        # Load Yolo
        yolo8_weights: str = self.get_parameter('yolo_weights').get_parameter_value().string_value
        self.yolo8 = YOLO(yolo8_weights)

    def run_yolo(self, image) -> dict[int, list]:
        """
        Runs Yolo on the image and returns the bounding boxes.

        Args:
            image: Image to run Yolo on

        Returns:
            list[list]: list of bounding boxes.
        """
        conf_threshold: float = self.get_parameter('yolo_conf_threshold').get_parameter_value().double_value
        # YoloBBoxs: [x1, y1, x2, y2, confidence, class]
        yolo_bboxs: list = self.yolo8(image, conf=conf_threshold)
        # BBoxs: [x, y, w, h, tracker_id, confidence, class]
        # BBoxes get initilaized with a tracker_id of -1
        return [
            [int(box[0]), int(box[1]), int(box[2] - box[0]), int(box[3] - box[1]), float(box[4]), int(box[5])] 
            for box in yolo_bboxs[0].boxes.data
        ]

    def onImage(self, msg: CompressedImage):
        """
        Image Callback: Runs Yolo on the image and publishes the results

        Args:
            msg (CompressedImage): Image message
        """

        # Start timer of processing
        timers: dict[str, Any] = {}

        def take_time(name: str):
            timers[name] = time.time()

        take_time("start")

        # Convert image to OpenCV format
        image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        take_time("post_cv_bridge")
        
        bboxs = self.run_yolo(image)
        take_time("post_yolo")
        
        """publish the bounding boxes"""
        self.publishBBoxs(bboxs, msg, image)

        take_time("end")
        """Prints the processing time of each step"""
        self.get_logger().info(
            '--------------------------------------------------------------------\n'+\
            f'Roun      ({(1E3 * (timers["end"] - timers["start"])):.1f}ms) \n'+\
            f'Bridge    ({(1E3 * (timers["post_cv_bridge"] - timers["start"])):.1f}ms) \n'+\
            f'Inference ({(1E3 * (timers["post_yolo"] - timers["post_cv_bridge"])):.1f}ms) \n'+\
            f'Rest      ({(1E3 * (timers["end"] - timers["post_yolo"])):.1f}ms) \n'+\
            '--------------------------------------------------------------------\n'
        )

    def publishBBoxs(self, bboxs, img_msg, img):
        """
        Publishes the bounding boxes of the detected objects as Polygons

        Args:
            bboxs (list[dict[int, list]]): List of bounding boxes
            img_msg (CompressedImage): Image message to get the timestamp and frame id from
            img (np.array): Image to overlay the bounding boxes on
        """
        def construct_polygon(bbox) -> Polygon:
            return Polygon(
                id=-1,
                color=bbox[5],
                polygon=GeoPolygon(
                    points = [Point32(x=float(bbox[0]),         y=float(bbox[1])        ),
                              Point32(x=float(bbox[0]+bbox[2]), y=float(bbox[1])        ),
                              Point32(x=float(bbox[0]+bbox[2]), y=float(bbox[1]+bbox[3])),
                              Point32(x=float(bbox[0]),         y=float(bbox[1]+bbox[3]))]
                )
            )
        """overlay colored bounding boxes on image"""
        if self.get_parameter('overlay_bbox').get_parameter_value().bool_value:
            import random
            for bbox in bboxs:
                color: tuple = self.colorByClass[bbox[5]]
                cv2.putText(img, f'{bbox[5]:.0%}', (bbox[0], bbox[1]-2), cv2.FONT_HERSHEY_SIMPLEX,.4, (0,0,0), 1, cv2.LINE_AA)
                cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), color, 2)

            self.publisher["bbox_over"].publish(self.cv_bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg'))

        stamped_polygons: PolygonsStamped = PolygonsStamped()
        stamped_polygons.header.stamp = img_msg.header.stamp
        
        stamped_polygons.header.frame_id = img_msg.header.frame_id+''
        stamped_polygons.polygons = [construct_polygon(bbox) for bbox in bboxs]
        
        self.publisher["stamped"].publish(stamped_polygons)

def main(args=None):
    rclpy.init(args=args)
    node = ConeTrackingNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.video_writer.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
