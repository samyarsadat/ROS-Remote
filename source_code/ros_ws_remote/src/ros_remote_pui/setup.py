from setuptools import find_packages, setup
from ros_remote_pui.config import PROGRAM_VERSION
package_name = "ros_remote_pui"

setup(
    name=package_name,
    version=PROGRAM_VERSION,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="The PUI (Physical User Interface) package for the ROS Remote Project.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={},
)
