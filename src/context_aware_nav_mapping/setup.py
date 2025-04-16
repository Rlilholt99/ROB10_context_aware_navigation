from setuptools import find_packages, setup

package_name = 'context_aware_nav_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'semantic_mapping = context_aware_nav_mapping.semantic_mapping_node:main'
        ],
    },
)
