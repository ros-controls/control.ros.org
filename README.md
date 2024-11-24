# Status

[![Build & Deploy Page](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml/badge.svg)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-make-page.yml)
[![Broken Links](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-check-links.yml/badge.svg?branch=rolling)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-check-links.yml)
[![Sphinx Warnings](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-check-warnings.yml/badge.svg?branch=rolling)](https://github.com/ros-controls/control.ros.org/actions/workflows/sphinx-check-warnings.yml)

# control.ros.org
https://control.ros.org/

This folder holds the source and configuration files used to generate the
[ros2_control documentation](https://control.ros.org) web site. The current test version of the documentation can be found [here](https://ros-controls.github.io/control.ros.org/).
We use [sphinx](https://www.sphinx-doc.org/en/master/) for single version and [sphinx-multiversion](https://holzhaus.github.io/sphinx-multiversion/master/index.html#) for the multi version build of our documentation.
The doc files themselves are written in [restructuredtext format](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html) (*.rst).

# Structure and build commands
The documentation files for [ros2_control](https://github.com/ros-controls/ros2_control), [ros2_controllers](https://github.com/ros-controls/ros2_controllers) and [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) are located in the respective repositories themselves (called subrepositories from now on).
They have to be included inside the `doc` folder.
There are `make` commands available which automate the process of building and inclusion of the subrepositories for you.
**NOTE**: In `spinx-multiverison` changes in the documentation are only visible after committing them. If you want to check them before committing, you can build a single version of the docs.
* ```make html``` - Builds a single version, changes are immediate visible. You have to include the subrepositories yourself.
* ```make html-all-subrepos``` - Builds a single version, changes are immediate visible. All subrepositories are automatically included.
* ```make multiversion``` - Builds multiversion version, changes are only visible after commit. **Make sure to commit everything before running!**
* For each command, a  ```make <command>-with-api``` exists, which in addition builds the `doxygen` api.

# Fetch reviewer stats
First, you need to fetch the reviewer stats from ros2_control org. To do so, you need to have a github token with the `repo` scope. Then run

```bash
export GITHUB_TOKEN=<your token>
python3 ./make_help_scripts/create_pr_stats.py
```
which will create `~/reviews/reviewers_stats_with_graph.html`. Then you can build the documentation as usual, it will copy the file from this folder.

# Build Instructions:
1. If you are running inside a docker container, be sure to open a port so the website can be accessed.
2. Install doxygen and graphviz: `sudo apt install doxygen graphviz`
3. `python3 -m pip install -r requirements.txt`
4. Install generate_parameter_library>0.3.3. If it is not installed as a ROS 2 package already, install it as python module from source
```bash
cd
git clone https://github.com/PickNikRobotics/generate_parameter_library.git
cd generate_parameter_library/generate_parameter_library_py/
python3 -m pip install .
```
5. Building of the documentation. Depends on what you want to do. See either single version or multiversion below.

If you want to see results run: `python3 -m http.server --directory <path_to_control.ros.org>/_build/html <port>` and then open a browser to `localhost:<port>`
 (Or just open `_build/html/index.html` in your browser.)

### Single version
4. a) Run `make html-all-subrepos` inside control.ros.org.
###
4. b) Checkout the relevant branches for control.ros.org and ros2_control.
Either checkout ros2_control inside `control.ros.org/doc/` or softlink it there. If you want to symlink it, clone it to any location you like.
Make sure you are inside the `control.ros.org/doc/` folder and run `ln -s <path to ros2_control> .`
6. Run `make html` inside control.ros.org.

### Multiversion
4. Run `make multiversion` inside control.ros.org.
##### A note on how it works:
The building of multiversion consists of several steps. The scripts are located inside the `make_help_scripts` folder.
1. First for each branch the subrepositories are cloned into the `doc/folder` and the corresponding branch is checked out and a temporary commit is created on the branch. This is needed, so that the documentation files located in the subrepositories are visible to multiversion.
2. Then the documentation is built using `sphinx-multiversion`
3. After the creation, each branch is checked out again and the temporary commits are deleted.

For `make multiversion-with-api` additionally, the api repository (ros2_control) is pulled and the corresponding branch is checked out. Then the api is created with `doxygen` and copied to the build directory.
**Note:** If the building fails it is possible that the temporary commits are not deleted correctly. You then have to check the branches manually an possibly clean up by deleting the temporary commits.
##### How to change:
If you want to change the deployed branches you have to change `conf.py` and `make_help_scripts/deploy_defines`.
#####
conf.py
* Change ros_distro, distro_title, distro_title_full and repos_file_branch.
* Add/remove whitelisting: smv_branch_whitelist,smv_released_pattern, ...
* possibly add your version/branch to: distro_full_names and branch_distro (in smv_rewrite_configs)
####
deploy_defines
* Change branch_version

To add subrepositories change subrepo_url

# Testing GitHub workflows:
If you want to modify or otherwise test the GitHub actions workflow locally without having to push every time to see the result,
you can use the [act](https://github.com/nektos/act) software package to run the workflows locally. You will need to use a GitHub token
as the workflows pulls external repositories. You can create one with only repo:public_repo rights [here](https://github.com/settings/tokens).

Once you have installed act and obtained a token for your user, run from within a local copy of this repository `act -s GITHUB_TOKEN -j <build-deploy or check-build>` and enter your token in the prompt. Be sure to replace the `<>` with one of the two workflow names.
