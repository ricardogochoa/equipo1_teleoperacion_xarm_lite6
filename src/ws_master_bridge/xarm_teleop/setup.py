from setuptools import setup
import os
from glob import glob

package_name = "xarm_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch",
         glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "master_node   = xarm_teleop.master_node:main",
            "slave_node    = xarm_teleop.slave_node:main",
            "udp_bridge_A  = xarm_teleop.udp_bridge_A:main",
            "udp_bridge_B  = xarm_teleop.udp_bridge_B:main",
        ],
    },
)
