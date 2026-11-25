from glob import glob
import os

from setuptools import find_packages, setup


package_name = "yahboom_r2_teleop"

setup(
    name=package_name,
    version="1.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*.html")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Chi Wei, Yeh",
    maintainer_email="yehchiwei.work@gmail.com",
    description="Joystick-controlled RTAB-Map bringup for the Yahboom R2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "r2_joy_teleop = yahboom_r2_teleop.r2_joy_teleop:main",
        ],
    },
)
