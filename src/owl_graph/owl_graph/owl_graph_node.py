import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from context_aware_nav_interfaces.srv import OwlLookup
import owlready2 as owl
import os
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict



class owl_graph_node(Node):
    def __init__(self):
        super().__init__('owl_graph_node')
        self.create_service(OwlLookup, 'owl_graph', self.owl_graph_callback)
        owl_path = self.get_owl_graph_path()
        self.owl_graph = owl.get_ontology(owl_path).load()



    def owl_graph_lookup(self,entity_names: list, relation: str = 'located_in') -> str:
        """
        Infers a single location based on the object or need labels found in the input list.
        First, it attempts to match Objects in the ontology; if none are found,
        it falls back to matching Needs (via objects that fulfill those needs).
        """
        inferred_locations = defaultdict(int)

        # First pass: try matching Objects directly
        for word in entity_names:
            normalized = word.strip().lower()
            for obj in self.owl_graph.Object.instances():
                if obj.name.lower() == normalized:
                    for loc in getattr(obj, relation, []):
                        inferred_locations[loc.name] += 1

        # Return top location if any object-based matches found
        if inferred_locations:
            return max(inferred_locations, key=inferred_locations.get)

        # Fallback: try matching Needs via Objects that fulfill them
        needs_matched = []
        for word in entity_names:
            normalized = word.strip().lower()
            for need in self.owl_graph.Need.instances():
                if need.name.lower() == normalized:
                    needs_matched.append(need)

        # Aggregate locations from objects fulfilling the matched needs
        for need in needs_matched:
            for obj in self.owl_graph.Object.instances():
                if need in getattr(obj, 'fulfills', []):
                    for loc in getattr(obj, relation, []):
                        inferred_locations[loc.name] += 1

        if inferred_locations:
            return max(inferred_locations, key=inferred_locations.get)

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
        