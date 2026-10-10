from setuptools import setup
package_name = 'jb_apriltag_align'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/align.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='jb', maintainer_email='3248428889@qq.com',
    description='最小视觉闭环', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'align_node = jb_apriltag_align.align_node:main',
    ]},
)
