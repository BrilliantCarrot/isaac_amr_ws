from setuptools import find_packages, setup
import os
from glob import glob

package_name = "manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "moveit_config"), glob("moveit_config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyj",
    maintainer_email="leeyj950322@gmail.com",
    description="Stationary arm motion bringup and pick-place scaffolding for the Isaac AMR mobile manipulator.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "franka_basic_motion_node = manipulation.franka_basic_motion_node:main",
            "follow_joint_trajectory_to_joint_state_server = manipulation.follow_joint_trajectory_to_joint_state_server:main",
        ],
    },
)
