import rclpy
import rclpy.duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from context_aware_nav_interfaces.msg import ObjectLocalPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy



class SemanticMapping(Node):
    def __init__(self):
        super().__init__('semantic_mapping_node')
        self.get_logger().info('Semantic mapping node has been started.')
        self.tf_business = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listenter = TransformListener(self.tf_buffer,self)
        self.test = ObjectLocalPose()
        self.is_robot_moving = False

        self.object_poses = []
        self.object_labels = []

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.create_subscription(ObjectLocalPose,"/object_local_pose",self.localPoseCallback,qos)
        self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self.amclPoseCallback,10)
        self.create_subscription(TwistStamped,"/mobile_base_controller/cmd_vel_out",self.cmd_vel_callback,10)
        self.marker_pub = self.create_publisher(Marker,"/object_pose",10)
        self.marker_array_pub = self.create_publisher(MarkerArray,"/multiple_object_poses",10)
        self.last_time = self.get_clock().now()
        self.create_timer(5.0, self.log_is_robot_moving)


    def cmd_vel_callback(self,msg):
        if msg.twist.linear.x != 0.0 or msg.twist.angular.z != 0.0:
            self.is_robot_moving = True
            self.last_time = self.get_clock().now()

        elif (self.get_clock().now() - self.last_time).nanoseconds >= 5 *1e9: 
            self.is_robot_moving = False

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
            return

        new_poses = []
        print(len(msg.object_pose))
        for obj in msg.object_pose:
            new_poses.append(self.transformObjectPose(obj,tf))


        if not self.is_robot_moving:
            self.visualizeObjectPose(new_poses)
            for obj, label in zip(new_poses,msg.object_labels):
                self.storeLocations(obj,label)


    def amclPoseCallback(self,msg):
        self.get_logger().info('testing amcl')

    def transformObjectPose(self, object_pose, tf):
        # Transform the object pose using the provided transform

        transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose,tf)
        return transformed_pose

    def storeLocations(self, object_pose, object_label):
        if len(self.object_poses) == 0:
            self.get_logger().info("first object added")
            self.object_poses.append(object_pose)
            self.object_labels.append(object_label)
            return
        for i in range(len(self.object_poses)):
            distance = self.compute_object_pose_euclidean_distance(object_pose,self.object_poses[i])

            if distance < 1.5 and self.object_labels[i] == object_label:
                return
            else:
                pass
        self.object_poses.append(object_pose)
        self.object_labels.append(object_label)
            
    

    def compute_object_pose_euclidean_distance(self, object_pose1, object_pose2):
        # Compute the Euclidean distance between two object poses
        dx = object_pose1.position.x - object_pose2.position.x
        dy = object_pose1.position.y - object_pose2.position.y
        dz = object_pose1.position.z - object_pose2.position.z
        return (dx**2 + dy**2 + dz**2)**0.5


        
    def log_is_robot_moving(self):
        if self.is_robot_moving:
            self.get_logger().info('Robot is moving')
        else:
            self.get_logger().info('Robot is not moving')

            self.get_logger().info(f'Object poses: {len(self.object_poses)}')



    def visualizeObjectPose(self, object_pose):

        markerArray = MarkerArray()
        # Create a Marker message to visualize the object pose
        i = 0
        for obj in object_pose:

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "object_pose"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = obj
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            i +=1


            markerArray.markers.append(marker)

        # Publish the marker
        self.marker_array_pub.publish(markerArray)
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