import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster



class SemanticMapping(Node):
    def __init__(self):
        super().__init__('semantic_mapping_node')
        self.get_logger().info('Semantic mapping node has been started.')
        self.tf_broadcaster = TransformBroadcaster(self)

    def run(self):
        self.get_logger().info('Running semantic mapping node...')
        # Add your mapping logic here


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()