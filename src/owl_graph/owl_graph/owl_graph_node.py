import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from context_aware_nav_interfaces.srv import OwlLookup
import owlready2 as owl
import os
from ament_index_python.packages import get_package_share_directory



class owl_graph_node(Node):
    def __init__(self):
        super().__init__('owl_graph_node')
        self.create_service(OwlLookup, 'owl_graph', self.owl_graph_callback)
        owl_path = self.get_owl_graph_path()
        self.owl_graph = owl.get_ontology(owl_path).load()

        print(self.owl_graph.Object.instances())


    def owl_graph_lookup(self, entity_name : str, relation='located_in'):
        words = [word.strip().lower() for word in entity_name.split(',')]
        inferred_locations = set()

        for word in words:
            for obj in self.owl_graph.Object.instances():
                if obj.name.lower() == word:
                    print(f"Matched object: {obj.name}")
                    # Check the specified relation (e.g., 'located_in')
                    for location in obj.located_in:
                        inferred_locations.add(location.name)

        if inferred_locations:
            print(f"Inferred locations: {inferred_locations}")
            return ', '.join(inferred_locations)
        else:
            print("No locations inferred.")
            return "No locations inferred."



    def owl_graph_callback(self, request, response):
        response = OwlLookup.Response()
        print("Received request: ", request.input)

        print(self.owl_graph_lookup(request.input))

        response.output = "Hello from owl_graph_node!"
        return response

    def get_owl_graph_path(self):
        package_name = 'owl_graph'
        package_share_directory = get_package_share_directory(package_name)
        owl_graph_path = os.path.join(package_share_directory, 'ontology_graphs', 'robot_ontology.owl')
        return owl_graph_path


def main(args=None):
    rclpy.init(args=args)
    node = owl_graph_node()
    try:

        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    node.destroy_node()
    rclpy.shutdown()
        