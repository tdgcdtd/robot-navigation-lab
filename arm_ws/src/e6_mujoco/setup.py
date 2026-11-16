from glob import glob

from setuptools import setup

package_name = "e6_mujoco"

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
    maintainer="tdgcdtd",
    maintainer_email="221798772+tdgcdtd@users.noreply.github.com",
    description="Bridge between MoveIt and a MuJoCo simulation of the Dobot E6",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = e6_mujoco.bridge_node:main",
        ],
    },
)
