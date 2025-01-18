from setuptools import find_packages, setup
from test_camera_publisher.config import PROGRAM_VERSION
package_name = "test_camera_publisher"

setup(
    name=package_name,
    version=PROGRAM_VERSION,
    packages=find_packages(exclude=["test"]),
    include_package_data=False,
    package_data={package_name: ["test_images/*.png"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"])
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="Test CompressedImage publisher for the ROS Remote Project.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"publisher_node = {package_name}.main:main"
        ],
    },
)
