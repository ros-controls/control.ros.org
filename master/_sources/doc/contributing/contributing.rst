Contributing
=============

First, thank you for considering contributing to the ros2_control project.
As an open-source project, we welcome each contributor, regardless of their background and experience.
To reduce the entropy of the universe and our vivid, open, and collaborative environment, we have set up some standards and methods for contributions.


Finding contributions to work on
--------------------------------
Looking at the existing issues is a great way to find something to contribute on.
We created a project board to help you find issues that are good for newcomers, see the `Contributing Board <https://github.com/orgs/ros-controls/projects/11>`__.

Pull Requests
-------------

Before sending us a pull request, please ensure that:

1. Limited scope. Your PR should do one thing or one set of things. Avoid adding "random fixes" to PRs. Put those on separate PRs.

2. Give your PR a descriptive title. Add a short summary, if required.

3. Make sure the pipeline is green.

4. Don't be afraid to request reviews from maintainers.

5. New code = new tests. If you are adding new functionality, always make sure to add some tests exercising the code and serving as live documentation of your original intention.

**To send us a pull request, please:**

1. Fork the repository.
2. Modify the source; please focus on the specific change you are contributing. If you also reformat all the code, it will be hard for us to focus on your change.
3. Ensure local tests pass. (``colcon test`` and ``pre-commit run`` (requires you to install pre-commit by ``pip3 install pre-commit``)
4. Commit to your fork using clear commit messages.
5. Send a pull request, answering any default questions in the pull request interface.
6. Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation.

GitHub provides additional documentation on `forking a repository <https://help.github.com/articles/fork-a-repo/>`__ and
`creating a pull request <https://help.github.com/articles/creating-a-pull-request/>`__.


Rules for the repositories and process of merging pull requests
----------------------------------------------------------------

This section targets maintainers, but you are also welcome to read it to understand the process of how we handle PRs in our organization.
This guideline is especially applicable for the following repositories:

* ros2_control,
* ros2_controllers,
* ros2_control_demos.

Please keep the following in mind:

1. Please work from your fork when submitting PR. That way, we are keeping the main repo clean from feature branches.

2. Each PR should have all checks satisfied before they can be considered for merging.

3. Each PR must be approved by two maintainers (explicitly, please!). Only exceptions are PR's from other active maintainers in the repository, where one approval backed up with traceable discussion is sufficient.

  .. note::

    Even if you are not a maintainer, you are still encouraged to review pull requests. This helps us increase the review pace and increase code quality. Also, you are very likely to find some issues/limitations nobody else is seeing.

4. There is no need to do "squash and merge" of commits to your PR. We will squash the commits when merging the PR into the head branch.

5. Please do not do "cowboy-style" PR merges over the weekend. It doesn't matter how trivial PR is. Give people a chance to do a proper review and comment on it.

6. Be aware of the impact a PR has and give other maintainers and contributors sufficient time for the review proportional to its impact. Ping them if necessary, repeatedly if necessary.


Writing documentation
----------------------

We use Sphinx with `Read The Docs theme <https://docs.readthedocs.io/en/stable/index.html>`__ for documentation.

General information is located in the `control.ros.org <https://github.com/ros-controls/ros2_control>`__ repository, while the documentation for the packages is written in the respective repositories.


Licensing
------------------------------------------
Any contribution that you make to this project will
be under the Apache 2 License, as dictated by that
`license <http://www.apache.org/licenses/LICENSE-2.0.html>`__:


  5. Submission of Contributions. Unless You explicitly state otherwise,
  any Contribution intentionally submitted for inclusion in the Work
  by You to the Licensor shall be under the terms and conditions of
  this License, without any additional terms or conditions.
  Notwithstanding the above, nothing herein shall supersede or modify
  the terms of any separate license agreement you may have executed
  with Licensor regarding such Contributions.


Repository structure and CI configuration
------------------------------------------

Each repository has two types of branches, development, and stable.
PR's should always be submitted against the development branch.
When PR is accepted, and there are no API and ABI changes to a stable branch, please open a new PR against the stable branch(es).
We use the following naming conventions for branches.

**Development branch**:

* Name: ``master``
* CI rule for merge:

  * must: ``semi-binary`` (working against development branch of ros2_control)
  * good: ``binary``      (working against the same stable branch of other ros2_control repositories)

* ``source`` build each day check against master branches of ROS 2

**Stable branches**:

* Name: ``<ros_distro>`` (e.g., foxy, galactic)
* CI rule for merge:

  * must: ``semi-binary`` (working against the same stable branch of other ros2_control repositories)
  * must: ``binary``    (working against released versions of ros2_control) - except for adding new non-braking features

* ``source`` build each day against distribution branches


CI configuration
----------------
Three build stages are checking the current and future compatibility of the framework.

1. ``binary`` - against released packages (main and testing) in ROS distributions. This Shows that direct local build is possible.

2. ``semi-binary`` - against released core ROS packages (main and testing), but the immediate dependencies are pulled from the source.
   This shows that local build with dependencies is possible, and if it fails there, we can expect that after the next package sync, we will not be able to build.

3. ``source`` - also core ROS packages are build from source. It shows potential issues in the mid future.


Documentation Usage
--------------------

.. raw:: html

   <iframe plausible-embed src="https://plausible.io/share/control.ros.org?auth=tvWI_5b9EWW3e12NsySNr&embed=true&theme=system&background=transparent" scrolling="no" frameborder="0" loading="lazy" style="width: 1px; min-width: 100%; height: 1600px;"></iframe>
   <div style="font-size: 14px; padding-bottom: 14px;">Stats powered by <a target="_blank" style="color: #2c3953; text-decoration: underline;" href="https://plausible.io">Plausible Analytics</a></div>
   <script async src="https://plausible.io/js/embed.host.js"></script>
