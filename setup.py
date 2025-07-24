from setuptools import find_packages, setup

package_name = 'da_vinci_tool_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', ['launch/med7_combined_simple.launch.py']),
        # Install all urdf files
        ('share/' + package_name + '/urdf', [
            'urdf/med7.xacro',
            'urdf/psm.base.xacro',
            'urdf/psm.classic.urdf.xacro',
            'urdf/psm.tool.xacro',
        ]),
        # Install all adaptor files and meshes
        ('share/' + package_name + '/urdf/adaptor', [
            'urdf/adaptor/adaptor.urdf',
        ]),
        ('share/' + package_name + '/urdf/adaptor/meshes', [
            'urdf/adaptor/meshes/Da_vinci_si_manipulator.stl',
        ]),
        # Install RViz config files
        ('share/' + package_name + '/rviz', [
            'rviz/my_robot_config.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suphasitpp',
    maintainer_email='suphasitpp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_solver = da_vinci_tool_integration.ik_solver:main',
            'interactive_target_marker = da_vinci_tool_integration.interactive_target_marker:main',
            'fk_query = da_vinci_tool_integration.fk_query:main',
        ],
    },
)
