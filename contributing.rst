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
