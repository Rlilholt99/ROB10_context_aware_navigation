import rclpy
import rclpy.duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from context_aware_nav_interfaces.msg import ObjectLocalPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray


class SemanticMapping(Node):
    def __init__(self):
        super().__init__('semantic_mapping_node')
        self.get_logger().info('Semantic mapping node has been started.')
        self.tf_business = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listenter = TransformListener(self.tf_buffer,self)
        self.test = ObjectLocalPose()

        self.create_subscription(ObjectLocalPose,"/object_local_pose",self.localPoseCallback,10)
        self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self.amclPoseCallback,10)
        self.marker_pub = self.create_publisher(Marker,"/object_pose",10)



    def run(self):
        self.get_logger().info('Running semantic mapping node...')
        # Add your mapping logic here

    def localPoseCallback(self,msg):
        now = rclpy.time.Time()
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='head_front_camera_depth_optical_frame',
                time=now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().info('Transform error: {}'.format(e))
            return

        if len(msg.object_pose) == 0:
            self.get_logger().info('No object detected')
            return
        new_pose = self.transformObjectPose(msg.object_pose[0],tf)

        self.visualizeObjectPose(new_pose)


    def amclPoseCallback(self,msg):
        self.get_logger().info('testing amcl')

    def transformObjectPose(self, object_pose, tf):
        # Transform the object pose using the provided transform

        transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose,tf)
        return transformed_pose

    def storeLocations(self, object_pose):
        # Store the object pose in a database or data structure
        self.get_logger().info('Storing object location: {}'.format(object_pose))

    def visualizeObjectPose(self, object_pose):

        # Create a Marker message to visualize the object pose
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_pose"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = object_pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        # Publish the marker
        self.marker_pub.publish(marker)
        pass

    def lookupOntology(self, object_list):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("stuff")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()