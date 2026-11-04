from glob import glob

from setuptools import setup

package_name = "so101_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "so101_driver = so101_teleop.so101_driver:main",
            "teleop = so101_teleop.teleop:main",
            "hapkit = so101_teleop.hapkit:main",
            "camera = so101_teleop.camera:main",
            "find_ports = so101_teleop.tools:find_ports",
            "find_cameras = so101_teleop.tools:find_cameras",
            "release = so101_teleop.tools:release",
        ],
    },
)
