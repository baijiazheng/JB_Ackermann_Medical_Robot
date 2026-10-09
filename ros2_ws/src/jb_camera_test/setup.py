from setuptools import setup

package_name = 'jb_camera_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament 索引：告诉 ROS2 这里有包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml 也一起安装
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        ('share/' + package_name + '/launch', ['launch/camera_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jb',
    maintainer_email='3248428889@qq.com',
    description='摄像头测试节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # ★ 关键：把 Python 脚本注册成可执行命令
        # 格式：'命令名 = 包名.模块名:函数名'
        # 装完后可以直接跑 ros2 run jb_camera_test camera_test
        'console_scripts': [
            'camera_test = jb_camera_test.camera_test:main',
            'camera_sub  = jb_camera_test.camera_sub:main',
        ],
    },
)
