import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from context_aware_nav_interfaces.srv import LocationLookup
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import spacy

class NLPCommandProcessor(Node):
    def __init__(self):
        super().__init__('nlp_command_processor')
        self.subscription = self.create_subscription(
            String, 'high_level_command', self.command_callback, 10)
        self.nlp = spacy.load("en_core_web_sm")
        self.get_logger().info("NLP Command Processor Node Started")
        self.callback_group_client = ReentrantCallbackGroup()
        self.location_lookup_client = self.create_client(LocationLookup,'/semantic_map_server/nav_to_location',callback_group=self.callback_group_client)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()


        self.navigate_to_pose_client = ActionClient(self,NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group_client)




    def navigate_to_pose(self, goalPose,behaviorTree=''):

        request = NavigateToPose.Goal()
        request.pose = goalPose
        request.behavior_tree = behaviorTree

        future = self.navigate_to_pose_client.send_goal_async(request)
        future.add_done_callback(self.nav_done_callback)
        self.get_logger().info(f'Sending navigation goal to {goalPose.pose.position.x}, {goalPose.pose.position.y}')


    def nav_done_callback(self,future):
        self.get_logger().info("nav is done")

    def command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f"Received command: {command_text}")

        # Process command
        parsed_command = self.parse_command(command_text)


    def parse_command(self, text):
        doc = self.nlp(text)
        location = None
        action = None
        locationTokes = []
        

        for token in doc:
            if token.pos_ in ["NOUN", "PROPN"]:
                locationTokes.append(token.text)
            if token.pos_ == "VERB":
                action = token.text
        



        location = " ".join(locationTokes)  # Output: living room


        if action and location:

            self.get_logger().info(f'lookup location {location}')
            self.lookupLocation(location)
            return "Parsed command: " + action + " to " + location
        return "Unable to parse command"

    def send_nav_goal(self,goalPose):
        self.nav.goToPose(goalPose)

    def lookupLocation(self,locationString):
        
        if not self.location_lookup_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, waiting again...')
        request = LocationLookup.Request()
        request.input = locationString

        future = self.location_lookup_client.call_async(request)
        future.add_done_callback(self.handle_location_response)



    def handle_location_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Type of response.output: {type(response.output)}")
            self.get_logger().info(f"response.output: {response.output}")
            goalPose = PoseStamped()
            goalPose.header.frame_id = 'map'
            goalPose.header.stamp = self.nav.get_clock().now().to_msg()
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
