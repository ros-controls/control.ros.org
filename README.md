# Status

[![Build & Deploy Page](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml/badge.svg)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml)

# control.ros.org
https://control.ros.org/

This folder holds the source and configuration files used to generate the
[ros2_control documentation](https://control.ros.org) web site.

**NOTE**: Current test version of the documentation can be found [here](https://ros-controls.github.io/control.ros.org/).

Dependencies for Build:
* `sudo apt install python3-pip`
* `pip3 install sphinx sphinx-rtd-theme`


Alternatively, follow the [Sphinx installation guide](https://www.sphinx-doc.org/en/master/usage/installation.html).

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.

