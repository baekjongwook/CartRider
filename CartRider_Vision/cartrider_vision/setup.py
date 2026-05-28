from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cartrider_vision'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="baek",
    maintainer_email="bjw9458@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "vision = cartrider_vision.vision_node:main",
            "rs_aruco_node = cartrider_vision.rs_aruco_node:main",
            "rs_pcd_node = cartrider_vision.rs_pcd_node:main",
        ],
    },
)
