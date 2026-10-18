from glob import glob

from setuptools import setup

package_name = "inspire_hand_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS2 driver and manual control GUI for the Inspire RH56 dexterous hand",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver = inspire_hand_ros2.driver_node:main",
            "booster_bridge = inspire_hand_ros2.booster_bridge_node:main",
            "hand_gui = inspire_hand_ros2.hand_gui:main",
        ],
    },
)
