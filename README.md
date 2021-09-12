# Status

[![Build & Deploy Page](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml/badge.svg)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml)

# control.ros.org
https://control.ros.org/

This folder holds the source and configuration files used to generate the
[ros2_control documentation](https://control.ros.org) web site.

**NOTE**: Current test version of the documentation can be found [here](https://ros-controls.github.io/control.ros.org/).

# Build Instructions:
1. If you are running inside a docker container, be sure to open a port so the website can be accessed.
2. Checkout the relevant branches for control.ros.org and ros2_control.
    Note: Either checkout ros2_control inside control.ros.org or softlink it there.
          The build will expect ros2_control to be found within control.ros.org.
3. `sudo apt install doxygen graphviz`
4. `pip3 install sphinx sphinx_rtd_theme`
5. Run `make html` inside control.ros.org. This will also build the doxygen api documentation.
6. If you want to see results run: `python3 -m http.server --directory <path_to_control.ros.org>/_build/html <port>`
7. Open a browser to localhost:<port>

Alternatively, follow the [Sphinx installation guide](https://www.sphinx-doc.org/en/master/usage/installation.html).
# Testing GitHub workflows:
If you want to modify or otherwise test the GitHub actions workflow locally without having to push everytime to see the result,
you can use the [act](https://github.com/nektos/act) software package to run the workflows locally. You will need to use a GitHub token
as the workflows pulls external repositories. You can create one with only repo:public_repo rights [here](https://github.com/settings/tokens). 

Once you have installed act and obtained a token for your user, run from within a local copy of this repository `act -s GITHUB_TOKEN -j <build-deploy or check-build>` and enter your token in the prompt. Be sure to replace the `<>` with one of the two workflow names.
