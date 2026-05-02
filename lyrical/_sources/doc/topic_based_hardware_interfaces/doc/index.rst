:github_url: https://github.com/ros-controls/topic_based_hardware_interfaces/blob/{REPOS_FILE_BRANCH}/doc/index.rst

topic_based_hardware_interfaces
###############################

.. include:: ../joint_state_topic_hardware_interface/README.md
   :parser: myst_parser.sphinx_

cm_topic_hardware_component
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The controller_manager topic System implements a ros2_control ``hardware_interface::SystemInterface`` that subscribes
to topics of type ``pal_statistics_msgs::msg::StatisticsNames`` and ``pal_statistics_msgs::msg::StatisticsValues``,
and sets its state interface to the received values (if present).

Per default, the ros2_control controller manager publishes these topics to ``/controller_manager/introspection_data/names``
and ``/controller_manager/introspection_data/values``.

This component serves as a possibility to replay ROS bags and inject the states from a hardware component into the ros2_control stack.
For example, use ``ros2bag`` CLI to extract these two topics via

.. code:: bash

  ros2 bag play <bag_file> \
    --topics /controller_manager/introspection_data/names /controller_manager/introspection_data/values \
    --remap /controller_manager/introspection_data/names:=/<hardware_component_name>/names\
            /controller_manager/introspection_data/values:=/<hardware_component_name>/values

Note that with this setup the current time of your OS is used and not published from the ROS bag.
If you want to control the speed of playback, run

* your ros2_control_node with ``--ros-args -p --use_sim_time:=true``
* and the ``--rate`` and ``--clock`` options, for example

  .. code:: bash

    ros2 bag play <bag_file> --rate 2.0 --clock 100 \
      --topics /controller_manager/introspection_data/names /controller_manager/introspection_data/values \
      --remap /controller_manager/introspection_data/names:=/<hardware_component_name>/names\
              /controller_manager/introspection_data/values:=/<hardware_component_name>/values

ROS subscribers
----------------------------
* /<hardware_component_name>/names: ``pal_statistics_msgs::msg::StatisticsNames``
* /<hardware_component_name>/values:  ``pal_statistics_msgs::msg::StatisticsValues``

ros2_control section in URDF
----------------------------

.. code:: xml

  <ros2_control name="hardware_component_name" type="system">
    <hardware>
      <plugin>cm_topic_hardware_component/CMTopicSystem</plugin>
    </hardware>
    <joint name="joint_1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.2</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint_2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.3</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint_3">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.1</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
