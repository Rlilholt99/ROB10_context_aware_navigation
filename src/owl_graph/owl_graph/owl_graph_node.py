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



    def owl_graph_lookup(self, entity_names: list, relation='located_in'):
        """
        Infers a single location based on the object labels found in the input list.
        """
        inferred_locations = {}

        for word in entity_names:
            word = word.strip().lower()
            for obj in self.owl_graph.Object.instances():
                if obj.name.lower() == word:
                    print(f"Matched object: {obj.name}")
                    # Check the specified relation (e.g., 'located_in')
                    for location in obj.located_in:
                        if location.name in inferred_locations:
                            inferred_locations[location.name] += 1
                        else:
                            inferred_locations[location.name] = 1

        if inferred_locations:
            # Infer the most likely location based on the highest count
            most_likely_location = max(inferred_locations, key=inferred_locations.get)
            print(f"Inferred location: {most_likely_location}")
            return most_likely_location
        else:
            print("No locations inferred.")
            return "No locations inferred."


    def owl_graph_callback(self, request, response):
        response = OwlLookup.Response()
        print("Received request: ", request.input)
        location = self.owl_graph_lookup(request.input)
        self.get_logger().info(f"Inferred location: {location}")

        response.output.append(location) 
        self.get_logger().info('returning response')
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
        