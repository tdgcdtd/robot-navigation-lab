from glob import glob

from setuptools import setup

package_name = "e6_llm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tdgcdtd",
    maintainer_email="221798772+tdgcdtd@users.noreply.github.com",
    description="Project C: natural-language commands for the Dobot E6",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planner_node = e6_llm.planner_node:main",
            "text_input = e6_llm.text_input:main",
            "voice_input = e6_llm.voice_input:main",
        ],
    },
)
