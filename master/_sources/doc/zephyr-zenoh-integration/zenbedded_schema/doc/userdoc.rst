:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/zenbedded_schema/doc/userdoc.rst

.. _zenbedded_schema_userdoc:

zenbedded_schema
================

YAML-driven schema parser and C/C++ header generator for packed binary
interface descriptions. builds with ``colcon`` or
standalone ``CMake``.

The schema is the single source of truth for the wire format shared between
the ROS 2 host and the Zephyr firmware.


Schema YAML format
------------------

.. code:: yaml

   state_interfaces:
     <component_name>:
       <field_name>: <type>
   command_interfaces:
     <component_name>:
       <field_name>: <type>

Supported field types: ``float32``, ``float64`` / ``double``, ``int32``,
``uint64``, ``int16``, ``uint8``.


Generated output
----------------

For the following schema:

.. code:: yaml

   state_interfaces:
     motor_arm:
       position: float32
     pendulum_axis:
       position: float32
   command_interfaces:
     motor_arm:
       position: float32

The generator produces:

.. code:: c

   #define ZENBEDDED_STATE_BYTE_SIZE 8
   #define ZENBEDDED_COMMAND_BYTE_SIZE 4

   #pragma pack(push, 1)
   typedef struct {
     float motor_arm_position;
     float pendulum_axis_position;
   } zenbedded_state_t;

   typedef struct {
     float motor_arm_position;
   } zenbedded_command_t;
   #pragma pack(pop)


CLI usage
---------

.. code:: bash

   # colcon
   colcon build --packages-select zenbedded_schema
   source install/setup.bash
   generate_header --yaml interface_schema.yaml --output interface_data.h

   # standalone
   cmake -S . -B build && cmake --build build
   ./build/generate_header --yaml interface_schema.yaml --output interface_data.h


CMake integration
-----------------

.. code:: cmake

   find_package(zenbedded_schema REQUIRED)

   zenbedded_generate_header(
     YAML  interface_schema.yaml
     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/interface_data.h
     TARGET generate_interface_data
   )

   add_dependencies(my_target ${generate_interface_data})


API
---

.. code:: cpp

   namespace zenbedded
   {

   class InterfaceSchema
   {
   public:
     static InterfaceSchema from_yaml(const std::string & yaml_text);
     static InterfaceSchema from_yaml_file(const std::string & file_path);

     bool valid() const;
     std::string error() const;

     size_t state_buffer_size() const;
     size_t command_buffer_size() const;
     size_t total_state_interfaces() const;
     size_t total_command_interfaces() const;

     bool write_c_header(const std::string & output_path) const;

     double read_state_field(const uint8_t * buffer, size_t field_index) const;
     double read_command_field(const uint8_t * buffer, size_t field_index) const;
     void write_command_field(uint8_t * buffer, size_t field_index, double value) const;

     static size_t field_byte_size(FieldType type);
   };

   }
