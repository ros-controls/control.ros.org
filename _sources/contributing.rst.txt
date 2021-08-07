First, thank you for considering contributing to the ros2_control project.
As an open-source project, we welcome each contributor, regardless of their background and experience.
To reduce the entropy of the universe and our vivid, open, and collaborative environment, we have set up some standards and methods for contributions.


Pull Requests
==============

Requirements for pull requests are as follows:

1. Limited scope. Your PR should do one thing or one set of things. Avoid adding "random fixes" to PRs. Put those on separate PRs.

2. Give your PR a descriptive title. Add a short summary, if required.

3. Make sure the pipeline is green.

4. Don't be afraid to request reviews from maintainers.

5. New code = new tests. If you are adding new functionality, always make sure to add some tests exercising the code and serving as live documentation of your original intention.


Rules for the repositories and process of merging pull requests
================================================================

This section targets maintainers, but you are also welcome to read it to understand the process of how we handle PRs in our organization.
This guideline is especially applicable for the following repositories:

* ros2_control,
* ros2_controllers,
* ros2_control_demos.

Please keep the following in mind:

1. Please work from your fork when submitting PR. That way, we are keeping the main repo clean from feature branches.

2. Each PR should have all checks satisfied before they can be considered for merging.

3. Each PR must be approved by two maintainers (explicitly, please!). Only exceptions are PR's from other active maintainers in the repository, where one approval backed up with traceable discussion is sufficient.

   **NOTE**: If you are not a maintainer, you are still encouraged to review pull requests. This helps us increase the review pace and increase code quality. Also, you are very likely to find some issues/limitations nobody else is seeing.

4. Always do "squash and merge" and clean a commit message from comments like "fixup linters", "use pre-commit", "correct header", or "Address review comments" and similar. This means that each PR results in exactly one (1) commit on the main branch.

5. Please do not do "cowboy-style" PR merges over the weekend. It doesn't matter how trivial PR is. Give people a chance to do a proper review and comment on it.

6. Be aware of the impact a PR has and give other maintainers and contributors sufficient time for the review proportional to its impact. Ping them if necessary, repeatedly if necessary.



Writing documentation
======================

Each repository starts as a "part" of the documentation using ``#``.
For more details, see: https://thomas-cokelaer.info/tutorials/sphinx/rest_syntax.html#id8


.. _ros2_control: https://github.com/ros-controls/ros2_control
.. _ros2_controllers: https://github.com/ros-controls/ros2_controllers
.. _ros2_control_demos: https://github.com/ros-controls/ros2_control_demos



Repository structure and CI configuration
=========================================

Each repository has two types of branches, development, and stable.
PR's should always be submitted against the development branch.
When PR is accepted, and there are no API and ABI changes to a stable branch, please open a new PR against the stable branch(es).
We use the following naming conventions for branches.

**Development branch**:

  - Name: ``master``
  - CI rule for merge:
    - must: ``semi-binary`` (working against development branch of ros2_control)
    - good: ``binary``      (working against the same stable branch of other ros2_control repositories)
  - ``source`` build each day check against master branches of ROS 2

**Stable branches**:

  - Name: ``<ros_distro>`` (e.g., foxy, galactic)
  - CI rule for merge:
    - must: ``semi-binary`` (working against the same stable branch of other ros2_control repositories)
    - must: ``binary``    (working against released versions of ros2_control) - except for adding new non-braking features
  - ``source`` build each day against distribution branches


CI configuration
----------------
Three build stages are checking the current and future compatibility of the framework.
1. ``binary`` - against released packages (main and testing) in ROS distributions. This Shows that direct local build is possible.
1. ``semi-binary`` - against released core ROS packages (main and testing), but the immediate dependencies are pulled from the source.
   This shows that local build with dependencies is possible, and if it fails there, we can expect that after the next package sync, we will not be able to build.
1. ``source`` - also core ROS packages are build from source. It shows potential issues in the mid future.
