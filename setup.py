import os
from glob import glob
from setuptools import find_packages, setup

package_name = "auvc_challenge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Saad Ibn Seddik",
    maintainer_email="msis@blksail.ai",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arming_timer = auvc_challenge.arming_timer:main",
            "pressure2depth = auvc_challenge.pressure2depth:main",
            "depth_pid = auvc_challenge.depth_pid:main",
        ],
    },
)
