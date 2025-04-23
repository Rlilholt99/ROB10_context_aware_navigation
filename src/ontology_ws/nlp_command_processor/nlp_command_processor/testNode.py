import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import spacy

class NLPCommandProcessor(Node):
    def __init__(self):
        super().__init__('nlp_command_processor')
        self.subscription = self.create_subscription(
            String, 'high_level_command', self.command_callback, 10)
        self.publisher = self.create_publisher(String, 'parsed_command', 10)
        self.nlp = spacy.load("en_core_web_sm")
        self.get_logger().info("NLP Command Processor Node Started")
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        initPose = self.populatePoseStamped()
        initPose.pose.position.x = -1.4
        initPose.pose.position.y = -0.4
        
        self.nav.setInitialPose(initPose)




    def command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f"Received command: {command_text}")

        # Process command
        parsed_command = self.parse_command(command_text)

        # Publish parsed command
        parsed_msg = String()
        parsed_msg.data = parsed_command
        self.publisher.publish(parsed_msg)
        self.get_logger().info(f"Published parsed command: {parsed_command}")

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


        print(locationTokes)
        location = " ".join(locationTokes)  # Output: living room


        if action and location:
            goalPose = self.lookupLocation(location)
            self.send_nav_goal(goalPose)
        return "Unable to parse command"

    def send_nav_goal(self,goalPose):
        self.nav.goToPose(goalPose)

    def lookupLocation(self,locationString):
        goalPose = self.populatePoseStamped()


        match locationString:

            case 'living room':
                goalPose.pose.position.x = -1.5
                goalPose.pose.position.y = 2.5
                goalPose.pose.position.z = 0.0

                goalPose.pose.orientation.w = 1.0

            case 'kitchen':
                goalPose.pose.position.x = 0.5
                goalPose.pose.position.y = -0.5
                goalPose.pose.position.z = 0.0

                goalPose.pose.orientation.w = 1.0

            case 'bedroom':
                goalPose.pose.position.x = -0.5
                goalPose.pose.position.y = -0.5
                goalPose.pose.position.z = 0.0

                goalPose.pose.orientation.w = 1.0

        return goalPose

    def populatePoseStamped(self):
        goalPose = PoseStamped()
        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = self.nav.get_clock().now().to_msg()
        
        return goalPose


def main(args=None):
    rclpy.init(args=args)
    node = NLPCommandProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
