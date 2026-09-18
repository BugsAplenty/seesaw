import os
from glob import glob
from setuptools import setup

package_name = "seesaw"

# ament needs an empty marker file named exactly like the package; git cannot
# track empty files, so create it on demand. NOTE: data_files sources must stay
# RELATIVE paths -- colcon's ament_python task asserts on absolute ones.
here = os.path.dirname(os.path.abspath(__file__))
os.makedirs(os.path.join(here, "resource"), exist_ok=True)
marker_abs = os.path.join(here, "resource", package_name)
if not os.path.exists(marker_abs):
    open(marker_abs, "a").close()

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),  # relative!
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "serial", "numpy"],
    entry_points={
        "console_scripts": [
            "sc15_driver = seesaw.sc15_driver:main",
            "scan_to_cloud = seesaw.scan_to_cloud:main",
        ],
    },
)
