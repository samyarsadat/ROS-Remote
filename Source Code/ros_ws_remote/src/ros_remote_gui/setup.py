from setuptools import find_packages, setup
from ros_remote_gui.config import PROGRAM_VERSION
package_name = "ros_remote_gui"

setup(
    name=package_name,
    version=PROGRAM_VERSION,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    requires=["PySide6"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="The GUI package of the ROS Remote Project.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "remote_gui_node = ros_remote_gui.main:main"
        ],
    },
)