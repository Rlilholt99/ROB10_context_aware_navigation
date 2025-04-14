from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'owl_graph'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ontology_graphs'),glob('ontology_graphs/*.owl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lilholt',
    maintainer_email='rasmuslilholt@hotmail.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'owl_graph_node = owl_graph.owl_graph_node:main',
        ],
    },
)
