from setuptools import setup

package_name = "ender_lidar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/inf_map_compressor.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "bridge_node = ender_lidar_bridge.bridge_node:main",
            "goal_node = ender_lidar_bridge.goal_node:main",
            "map_relay_node = ender_lidar_bridge.map_relay_node:main",
        ]
    },
)
