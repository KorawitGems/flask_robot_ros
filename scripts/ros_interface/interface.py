import rospy
from tf import TransformerROS, transformations
from tf2_ros import Buffer, TransformListener, LookupException
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import MarkerArray
import numpy as np

class ROSInterface:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.tf_merROS = TransformerROS()
        self.tf_mation = transformations

        self.param_map_frame_ = rospy.get_param('/ros_interface/map_frame', "map")
        self.param_robot_frame_ = rospy.get_param('/ros_interface/robot_frame', "base_link")

        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('object_markers', MarkerArray, self.markers_callback)
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        self.map_msg = None
        self.map_origin_in_global_mat = np.identity(4)
        self.global_in_map_origin_mat = np.identity(4) # global is map frame
        self.robot_in_map_origin_mat = np.identity(4) # map_origin is grid map's origin frame
        self.goal_in_global_mat = np.identity(4)
        self.goal_pose_msg = PoseStamped()
        self.object_markers_msg = None
        self.object_in_map_origin_mat = np.identity(4)

    def markers_callback(self, markers_msg):
        self.object_markers_msg = markers_msg

    def map_callback(self, mapData_msg):
        self.map_msg = mapData_msg

    def get_global_to_map_origin_mat(self):
        try:
            self.map_origin_in_global_mat = self.tf_merROS.fromTranslationRotation(
                                (self.map_msg.info.origin.position.x, self.map_msg.info.origin.position.y, self.map_msg.info.origin.position.z), 
                                (self.map_msg.info.origin.orientation.x, self.map_msg.info.origin.orientation.y, self.map_msg.info.origin.orientation.z, self.map_msg.info.origin.orientation.w))
            # Use another method to speed up the inverse.
            #self.global_in_map_origin_mat = self.tf_mation.inverse_matrix(self.map_origin_in_global_mat)
            self.global_in_map_origin_mat[:3,:3] = np.transpose(self.map_origin_in_global_mat[:3,:3])
            self.global_in_map_origin_mat[:3,3] = np.dot(-np.transpose(self.map_origin_in_global_mat[:3,:3]),(self.map_origin_in_global_mat[:3,3]))
        except Exception as e:
            rospy.logwarn("Failed to calculate global_frame_in_map_origin: %s", str(e))

    def get_robot_to_map_origin_mat(self):
        try:
            robot_in_global = self.tf_buffer.lookup_transform(self.param_map_frame_, self.param_robot_frame_, rospy.Time(0),rospy.Duration(1.0))
            robot_in_global_mat = self.tf_merROS.fromTranslationRotation(
                                    (robot_in_global.transform.translation.x, robot_in_global.transform.translation.y, robot_in_global.transform.translation.z), 
                                    (robot_in_global.transform.rotation.x, robot_in_global.transform.rotation.y, robot_in_global.transform.rotation.z, robot_in_global.transform.rotation.w))
            self.robot_in_map_origin_mat = np.dot(self.global_in_map_origin_mat, robot_in_global_mat)
            #rospy.loginfo("Robot in map origin: %s", self.robot_in_map_origin_mat)
        except LookupException as e:
            rospy.logwarn("Failed to get transform between 'map' and 'base_link': %s", str(e))

    def publish_goal(self):
        self.goal_pose_msg.header.frame_id = 'map'
        self.goal_pose_msg.header.stamp = rospy.Time.now()
        self.goal_pose_msg.pose.position = Point(*self.tf_mation.translation_from_matrix(self.goal_in_global_mat))
        self.goal_pose_msg.pose.orientation = Quaternion(*self.tf_mation.quaternion_from_matrix(self.goal_in_global_mat)) # * is unpack list or tuple to each variable
        self.goal_publisher.publish(self.goal_pose_msg)

