import numpy as np
import rclpy
from rclpy.node import Node
from cat_msgs_ros.msg import Cones
from geometry_msgs.msg import Pose
import math

from cat_msgs_ros.msg import Walls, Cone
from geometry_msgs.msg import Point32

'''
TODO:
generelles Vorgehen:
    1. Durchlaufe Message und finde orange Cone
    2. Suche für Orange Cones partner
        2.1 Prüfe, ob überhaupt zwei vorhanden sind --> betrachte 10 (oder gibt es bessere Möglichkeit, eventuell auch weniger) grüne Cones im Umkreis
        2.2 Setze grüne mit orangen cones onLine methode ein, schaue ob passt
        2.3 Wenn gefunden, trage Verbindung in irgendeine Art von Liste ein --> welche Struktur eignet sich hierfür am Besten? 
        2.4 Wenn nicht gefunden, tue nichts
    3. Trage die gefundenen Wände in Array ein
'''     

class ColorIDs: 
    BLUE = 0
    GREEN = 1
    PINK = 2
    RED = 3
    YELLOW = 4

class Mode:
    Bremergy = 0
    Logistic = 1


class WallDetectorNode(Node):
    color = None

    # Quality of Service
    QoS = rclpy.qos.QoSProfile(
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        depth=1,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST
    )

    def __init__(self):
        super().__init__("WallDetector")

        self.declare_parameter("mode", "Logistic")
        self.declare_parameter("max_offset_angle", 0.2)#11.45 deg
        self.declare_parameter("max_middle_offset", 0.05)
        self.declare_parameter("max_wall_length", 1.0)

        #set mode and parameters (color of used cones; number of nearest neighbors in logistic mode)
        self.mode = getattr(Mode, self.get_parameter("mode").get_parameter_value().string_value)
        if self.mode == Mode.Bremergy:
            import json
            self.declare_parameter("wall_colors", "[\"YELLOW\", \"BLUE\"]")
            self.wall_colors = [getattr(ColorIDs, color) for color in json.loads(self.get_parameter("wall_colors").get_parameter_value().string_value)]
        else:
            import json
            self.declare_parameter("wall_pairs", "[[\"RED\",\"GREEN\"]]")
            self.wall_pairs = [[getattr(ColorIDs, color) for color in color_pair] for color_pair in json.loads(self.get_parameter("wall_pairs").get_parameter_value().string_value)]
            self.declare_parameter("num_neighbors", 4)

        self.get_logger().info("Starting Wall Detector in %s mode" % ("Bremergy" if self.mode == Mode.Bremergy else "Logistic"))

        # Subscribtions
        self.create_subscription(
            msg_type= Cones,
            topic='cones/cones',
            callback= self.cones_callback,
            qos_profile=self.QoS
        )

        # Publisher
        self.pub= {
            "walls": self.create_publisher(Walls, 'walls', 10)
        }
    
    def cones_callback(self, msg: Cones): 
        """
        Callback for cones topic

        Args:
            msg: Cones message
        
        Cone[] cones
        Cones consists of:
        Pose pose x,y
        int8 color
        int16 id
        """
        conesout = []
        wallsout = []
        nearestCones = self.findNearestCones(msg)


        def insertCone(cone) -> int:
            if cone in conesout:
                return conesout.index(cone)
            else:
                conesout.append(cone)
                return len(conesout) - 1

        def findWallInLogistic(cone, neighbors):
            for i, _ in enumerate(neighbors):
                other = _["cone"]

                for _ in neighbors[i+1:]:
                    other2 = _["cone"]
                    if self.onLine(other2.pose, other.pose, cone.pose):
                        # Found a wall
                        cone_index = insertCone(cone)
                        index1     = insertCone(other)
                        index2     = insertCone(other2)
                        wallsout.extend([cone_index, index1])
                        wallsout.extend([cone_index, index2])
                        return
   
        if self.mode == Mode.Logistic:
            for _ in nearestCones:
                findWallInLogistic(_["cone"], _["neighbors"])

        elif self.mode == Mode.Bremergy:
            for _ in nearestCones:
                cone = _["cone"] 
                neighbors = _["neighbors"]

                for _ in neighbors:
                    other = _["cone"]
                    distance = _["distance"]
                    index1 = insertCone(cone)
                    index2 = insertCone(other)
                    wallsout += [index1, index2]
        out = Walls(
            header = msg.header,
            cones = [Point32(
                x=cone.pose.position.x,
                y=cone.pose.position.y,
                z=cone.pose.position.z,
            ) for cone in conesout], 
            walls = wallsout
        )
        self.pub["walls"].publish(out)
        self.get_logger().info('Detected %d Walls [mode: %s]' % (int(len(wallsout)/2), self.mode))

    @staticmethod
    def distance(pos1, pos2):
        """
        Calculates the euclidean distance between two positions 
        (any objects that have x, y and z attributes)

        Args:
            pos1: first position
            pos2: second position

        Returns:
            euclidean distance between pos1 and pos2
        """

        distance_cat2cone= math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z-pos2.z)**2)
        return distance_cat2cone
    
    # TODO: 
    #   Funktion die erkennt ob Cones auf einer Linie
    #   Vielleicht eine Idee wenn man zwei grüne Cones nimmt und die miteinander verbindet, das man dann guckt ob sich ein roter Cone auf der Linie (+- Abweichung) befindet

    def onLine(self, cone1: Pose, cone2: Pose, middle_cone: Pose):
        """
        Checks if cone3 is on the line between cone1 and cone2

        Args:
            cone1 (Pose): pose of first cone
            cone2 (Pose): pose of second cone
            middle_cone (Pose): pose of the cone that is checked if it is on the line between the other two cones

        Returns:
            bool: whether cone3 is on the line between cone1 and cone2
        """
        
        c_1 = np.array([cone1.position.x, cone1.position.y])
        c_2 = np.array([cone2.position.x, cone2.position.y])
        c_1_c_2 = c_2 - c_1

        # wall cant be longer then max_wall_length
        if np.linalg.norm(c_1_c_2) > self.get_parameter("max_wall_length").value:
            return False

        middle = np.array([middle_cone.position.x, middle_cone.position.y])
        c_1_middle = middle - c_1

        # check if middle cone is actually in the middle between cone1 and cone2
        if abs(np.linalg.norm(c_1_middle) - np.linalg.norm(middle - c_2)) > self.get_parameter("max_middle_offset").value:
            return False
    
        
        #angle between c_1_middle and c_1_c_2        
        inner_angle = np.arccos(
                        np.clip(np.dot(c_1_middle, c_1_c_2) / (np.linalg.norm(c_1_middle) * np.linalg.norm(c_1_c_2)), -1, 1)
                      )
        return inner_angle < self.get_parameter("max_offset_angle").value
    
        # maxVerticalDistance = (WallDetectorNode.distance(cone1.position, cone2.position)) * 0.8#0.20
        # x1 = cone1.position.x
        # x2 = cone2.position.x
        # x3 = middle_cone.position.x
        # y1 = cone1.position.y
        # y2 = cone2.position.y
        # y3 = middle_cone.position.y
        # middle = (np.array([x3, y3, 0]))
        # c1 = np.array([x1,y1, 0])
        # c1_c2 = np.array([x2-x1, y2-y1, 0])
        # c1_middle = middle-c1
        # crossProduct = np.cross(c1_middle, c1_c2)
        # len_cross = np.sqrt(crossProduct[0]**2 + crossProduct[1]**2 + crossProduct[2]**2)
        # len_c1_c2 = np.sqrt(c1_c2[0]**2 + c1_c2[1]**2 + c1_c2[2]**2)
        # distance = len_cross/len_c1_c2
        # if (distance <= maxVerticalDistance):
        #     return True
        # return False

    # TODO:
    #   vier nächste Cones finden auslagern
    #   Von jedem Cone werden die 4 vom Abstand nächsten Cones herausgesucht und die ID und Entfernung ausgegeben.

    def findNearestCones(self, msg: Cones):
        """
        Finds the self.`num_nearest` nearest cones to each cone in the message

        Args:
            msg (Cones): message containing the cones

        Returns:
            list: list with a tuple for each cone containing that cone and a list with tuples of the distance to its nearest neighbors and the neighbor itself
        """
        def getNearestOfColor(cones: Cones, cone: Cone, color: ColorIDs, num_nearest: int):
            matches: list[dict[str, any]] = []
            for other in cones.cones:
                if other.color != color or other == cone:
                    continue
                matches.append({
                    "cone": other,
                    "distance": WallDetectorNode.distance(
                        cone.pose.position,
                        other.pose.position
                    )
                })
                
            # sort by distance
            matches.sort(key=lambda match: match["distance"])
            # only keep the `num_nearest` closest cones
            return matches[:num_nearest]

        neighbors: list[dict[str, any]] = []
        self.num_neighbors = self.get_parameter("num_neighbors").get_parameter_value().integer_value
        if self.mode == Mode.Logistic:
            for cone in msg.cones:
                other_color = None
                for wall_pair in self.wall_pairs:
                    if cone.color != wall_pair[0]:
                        continue
                    other_color = wall_pair[1]
                if not other_color:
                    continue
                neighbors.append({
                    "cone": cone,
                    "neighbors": getNearestOfColor(msg, cone, other_color, self.num_neighbors)
                })
            return neighbors
        elif self.mode == Mode.Bremergy:
            for cone in msg.cones:
                if cone.color not in self.wall_colors:
                    continue
                neighbors.append({
                    "cone": cone,
                    "neighbors": getNearestOfColor(msg, cone, cone.color, 2)
                })
            return neighbors
        raise Exception("Invalid Mode")


def main(args=None):
    rclpy.init()
    node = WallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
