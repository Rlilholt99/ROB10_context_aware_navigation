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


    def owl_graph_lookup(self, entity_name : str, relation='located_in'):
        name = entity_name.lower()

        for ind in self.owl_graph.Object.instances():
            if ind.name.lower() == name:
                print(f"Found individual: {ind.name}")
                for rel in ind.get_relations():
                    if rel.name == relation:
                        print(f"Relation: {rel.name}")
                        return rel



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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        