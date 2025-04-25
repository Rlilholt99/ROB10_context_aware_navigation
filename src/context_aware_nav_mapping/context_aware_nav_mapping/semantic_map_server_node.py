import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from context_aware_nav_interfaces.srv import LocationLookup
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
import json


class SemanticMapServerNode(Node):
    def __init__(self):
        super().__init__('semantic_map_server_node')
        self.get_logger().info('Semantic Map Server Node has been started.')

        self.visualization_map_pub = self.create_publisher(MarkerArray,'visualization_marker_array', 10)
        self.create_service(LocationLookup,
            'semantic_map_server/nav_to_location', self.handle_location_lookup)


        self.filename = 'location.json'

        self.map = None
        self.load_map(self.filename)
        self.publish_visualization()
        


    def handle_location_lookup(self,request,response):
        response = LocationLookup.Response()
        self.get_logger().info(f'Request receieved {request.input}')

        response.output, response.location_found = self.compute_nav_location(request.input)
        return response
    
    def compute_nav_location(self,location):


        response = Pose()
        success = False
        for i, map in enumerate(self.map):
            if location == map['label'][0].lower():
                response.position.x = map['center_x']
                response.position.y = map['center_y']
                success = True
                
                return response, success
        return response, success



        # compute a navigation goal based on the rectangle which defines the area/location
        # The question is where to put the navigation goal
        # The first idea is to put it on the closest point of the rectangle
        # The second idea is to put it in the center of the rectangle
        # The third idea is to compute a random position witihin the rectangle
        # The first idea might place us inside a wall.
        # the second idea might place us inside an object
        # The third idea might place us inside an object
        # Maybe the correct step is to compute a position where as many of the objects are visible.
        pass

    def publish_visualization(self):
        if self.map is None:
            self.get_logger().warn('No map loaded. Cannot publish visualization.')
            return
        


        


        # Create a MarkerArray message
        marker_array = MarkerArray()



        for i,loc in enumerate(self.map):


            # Create a marker for each location in the map
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'semantic_map'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = loc['center_x']
            marker.pose.position.y = loc['center_y']
            marker.pose.position.z = 0.2
            marker.scale.x = loc['x']
            marker.scale.y = loc['y']
            marker.scale.z = 0.05 


            match loc['label'][0].lower(): 

                case "kitchen":
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                case "living_room":
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                case "bedroom":
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                case "No locations inferred.":
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
            marker.color.a = 0.2  # 20% opacity

            # Add the marker to the array
            marker_array.markers.append(marker)

        # Publish the MarkerArr
        self.visualization_map_pub.publish(marker_array)

        # Implement the logic to publish the map visualization here
    
    def load_map(self, map_path):
        self.get_logger().info(f'Loading map from {map_path}')
        with open(map_path, 'r') as f:
            self.map = json.load(f)
        f.close()

def main():
    rclpy.init()
    node = SemanticMapServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
