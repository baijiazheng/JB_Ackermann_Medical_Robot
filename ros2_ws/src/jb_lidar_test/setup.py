from setuptools import setup
package_name = 'jb_lidar_test'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar_test.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='jb', maintainer_email='3248428889@qq.com',
    description='雷达测试与安全节点', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'lidar_test  = jb_lidar_test.lidar_test:main',
        'lidar_safety= jb_lidar_test.lidar_safety:main',
    ]},
)
