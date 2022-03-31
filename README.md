# Status

[![Build & Deploy Page](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml/badge.svg)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml)


# control.ros.org
https://control.ros.org/

This folder holds the source and configuration files used to generate the
[ros2_control documentation](https://control.ros.org) web site. The current test version of the documentation can be found [here](https://ros-controls.github.io/control.ros.org/).  
We use [sphinx](https://www.sphinx-doc.org/en/master/) for our single version [sphinx-multiversion](https://holzhaus.github.io/sphinx-multiversion/master/index.html#) for the multi version build of our documentaion. The doc files themself are written in [restructuredtext format](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html) (*.rst).


# Structure and build commands
The documentation files for [ros2_control](https://github.com/ros-controls/ros2_control), [ros2_controllers](https://github.com/ros-controls/ros2_controllers) and [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) are located in the respective repositories themselves. They have to be included inside the `doc` folder. There are `make` commands available which automate the process of building and inclusion of the subrepositories for you.  
**NOTE**: In `spinx-multiverison` changes in the documentation are only visible after committing them. If you want to check them before committing, you can build a single version of the docs.   
* ```make html``` - Builds a single version, changes are immediate visible. You have to include the subrepositories yourself.
* ```make html-all-subrepos``` - Builds a single version, changes are immediate visible. All subrepositories are automatically included.
* ```make multiversion``` - Builds multiversion version, changes are only visible after commit. **Make sure to commit everything before running!**  
* For each command, a  ```make <command>-with-api``` exists, which in addition builds the `doxygen` api.


# Build Instructions:
1. If you are running inside a docker container, be sure to open a port so the website can be accessed.
2. Install doxygen and graphviz: `sudo apt install doxygen graphviz`
3. `python3 -m pip install -r requirements.txt`
4. Building of the documentation. Depends on what you want to do. See either single version or multiversion bleow.
 
If you want to see results run: `python3 -m http.server --directory <path_to_control.ros.org>/_build/html <port>` and then open a browser to `localhost:<port>`   
 (Or just open `_build/html/index.html` in your browser.)

### Single version
4. a) Run `make html-all-subrepos` inside control.ros.org. 
###
4. b) Checkout the relevant branches for control.ros.org and ros2_control. Either checkout ros2_control inside `control.ros.org/doc/` or softlink it there. If you want to symlink it, clone it to any location you like. Make sure you are inside the `control.ros.org/doc/` folder and run `ln -s <path to ros2_control> .`  
5. Run `make html` inside control.ros.org.

### Multiversion
4. Run `make multiversion` inside control.ros.org. 
##### A note on how it works:


# Testing GitHub workflows:
If you want to modify or otherwise test the GitHub actions workflow locally without having to push every time to see the result,
you can use the [act](https://github.com/nektos/act) software package to run the workflows locally. You will need to use a GitHub token
as the workflows pulls external repositories. You can create one with only repo:public_repo rights [here](https://github.com/settings/tokens). 

Once you have installed act and obtained a token for your user, run from within a local copy of this repository `act -s GITHUB_TOKEN -j <build-deploy or check-build>` and enter your token in the prompt. Be sure to replace the `<>` with one of the two workflow names.
