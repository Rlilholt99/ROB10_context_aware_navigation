import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Pose
from context_aware_nav_interfaces.srv import LocationLookup
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import spacy

from context_aware_nav_interfaces.msg import ObjectLocalPose
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import tf2_geometry_msgs

import json
import math

class NLPCommandProcessor(Node):
    def __init__(self):
        super().__init__('nlp_command_processor')
        self.subscription = self.create_subscription(
            String, 'high_level_command', self.command_callback, 10)
        self.nlp = spacy.load("en_core_web_trf")
        self.get_logger().info("NLP Command Processor Node Started")
        self.callback_group_client = ReentrantCallbackGroup()
        self.location_lookup_client = self.create_client(LocationLookup,
                                                         '/semantic_map_server/nav_to_location',
                                                         callback_group=self.callback_group_client)
        self.debug=True

        self.subscription = self.create_subscription(
            String, 'object_nav', self.command2_callback, 10)

        self.navigate_to_pose_client = ActionClient(self,NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group_client)

        self.approach_distance = 0.5
        self.last_object_detection = None

        self.object_detection_sub = self.create_subscription(ObjectLocalPose,'/object_local_pose', self.object_detection_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listenter = TransformListener(self.tf_buffer,self)


    def object_detection_callback(self, msg):
        self.last_object_detection = msg

    def navigate_to_pose(self, goalPose,behaviorTree=''):

        request = NavigateToPose.Goal()
        request.pose = goalPose
        request.behavior_tree = behaviorTree

        future = self.navigate_to_pose_client.send_goal_async(request)
        future.add_done_callback(self.nav_done_callback)
        self.get_logger().info(f'Sending navigation goal to {goalPose.pose.position.x}, {goalPose.pose.position.y}')


    def nav_done_callback(self,future):
        self.get_logger().info("nav is done")

    def command2_callback(self, msg):
        command_text = msg.data.lower()
        approach = self.navigate_to_object(command_text)

        if approach is None:
            self.get_logger().info("Object not found")
            return
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = self.get_clock().now().to_msg()
        goalPose.pose = approach
        self.navigate_to_pose(goalPose)

        # Process command
    def command_callback(self, msg):
        command_text = msg.data.lower()

        # Process command
        parsed_command = self.parse_command(command_text)

    def object_nav_callback(self, msg):
        command_text = msg.data.lower()

        approach_pose = self.navigate_to_object(command_text)
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = self.get_clock().now().to_msg()
        goalPose.pose = approach_pose
        self.navigate_to_pose(goalPose)
        # Process command

    def parse_command(self, text):
        doc = self.nlp(text)
        location = None
        action = None
        locationTokes = []
        

        for token in doc:
            self.get_logger().info(f'{token.pos_}')
            if token.pos_ in ["NOUN", "PROPN"]:
                locationTokes.append(token.text)
            if token.pos_ == "VERB":
                action = token.text
        



        location = " ".join(locationTokes)  # Output: living room


        if action and location:

            self.get_logger().info(f'lookup location {location}')
            self.get_logger().info(f'action {action}')
            if not self.debug:
                self.lookupLocation(location)
            return "Parsed command: " + action + " to " + location
        return "Unable to parse command"


    def lookupLocation(self,locationString):
        
        if not self.location_lookup_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, waiting again...')
        request = LocationLookup.Request()
        request.input = locationString

        future = self.location_lookup_client.call_async(request)
        future.add_done_callback(self.handle_location_response)


    def transformObjectPose(self, object_pose, tf):
        # Transform the object pose using the provided transform

        transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose,tf)
        return transformed_pose

    def navigate_to_object(self, object_name):

        goal = Pose()
        for i, object in enumerate(self.last_object_detection.object_labels):
            if object == object_name:
                goal = self.last_object_detection.object_pose[i]


                now = rclpy.time.Time()
                try:
                    # Transform the goal to the map frame
                    transform = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame='head_front_camera_depth_optical_frame',
                    time=now,
                    timeout=rclpy.duration.Duration(seconds=0.1))

                except TransformException as e:
                    return False

                goal = self.transformObjectPose(goal, transform)

                
                try:
                    # Transform the goal to the map frame
                    robot_transform = self.tf_buffer.lookup_transform(
                    target_frame='base_link',
                    source_frame='map',
                    time=now,
                    timeout=rclpy.duration.Duration(seconds=0.1))

                except TransformException as e:
                    return False

                robot_pose = Pose()

                robot_pose.position.x = robot_transform.transform.translation.x
                robot_pose.position.y = robot_transform.transform.translation.y
                robot_pose.position.z = robot_transform.transform.translation.z
                robot_pose.orientation.x = robot_transform.transform.rotation.x
                robot_pose.orientation.y = robot_transform.transform.rotation.y
                robot_pose.orientation.z = robot_transform.transform.rotation.z
                robot_pose.orientation.w = robot_transform.transform.rotation.w


                    
                approach_pose = self.compute_object_navigation_goal(robot_pose_input=robot_pose, obj_pose=goal)

                



                

                return approach_pose
            else:
                return None
    
    def compute_object_navigation_goal(self,robot_pose_input , obj_pose):
        """
        Compute a goal pose that is at a specified distance from the object along the line from robot to object.
        """
        # Assume robot at origin of map frame orientation 0
        # Vector from robot to object
        dx = obj_pose.position.x - robot_pose_input.position.x
        dy = obj_pose.position.y - robot_pose_input.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist <= self.approach_distance:
            # Already within range
            return obj_pose

        # Scale to approach distance
        scale = (dist - self.approach_distance) / dist
        approach_x = robot_pose_input.position.x + dx * scale
        approach_y = robot_pose_input.position.y + dy * scale

        approach = Pose()
        approach.position.x = approach_x
        approach.position.y = approach_y
        approach.position.z = obj_pose.position.z
        # Face the object
        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        approach.orientation.z = qz
        approach.orientation.w = qw
        return approach

    def handle_location_response(self, future):
        try:
            response = future.result()
            goalPose = PoseStamped()
            goalPose.header.frame_id = 'map'
            goalPose.header.stamp = self.get_clock().now().to_msg()
            goalPose.pose = response.output
            self.navigate_to_pose(goalPose)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NLPCommandProcessor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
