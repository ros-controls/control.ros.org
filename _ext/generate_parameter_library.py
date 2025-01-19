#!/usr/bin/env python3
# Copyright 2025 ros2_control development team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from docutils import nodes
from sphinx.util.docutils import SphinxDirective
from sphinx.util.nodes import nested_parse_with_titles
from docutils.statemachine import ViewList
import yaml
from generate_parameter_library_py.parse_yaml import (
    GenerateCode,
)
from generate_parameter_library_py.generate_markdown import (
    DefaultConfigMarkdown,
    ParameterDetailMarkdown,
    RuntimeParameterDetailMarkdown
)
import sphinx.util.logging

logger = sphinx.util.logging.getLogger(__name__)

def insert_additional_parameters_after(items, keys, insert_map):
  """
  Inserts additional parameters into the items and keys lists after the top level key match
  """
  for key in insert_map.keys():
      # Define the level you're looking for
      level_to_find = key.split('.')[0]

      # Reverse keys and find the index of the first occurrence of the level
      reversed_keys = list(reversed(keys))
      reversed_index = next((index for (index, item) in enumerate(reversed_keys) if item.split('.')[0] == level_to_find), None)

      if reversed_index is not None:
          # If the level is found, insert the insert_map after it in the original list
          index = len(keys) - 1 - reversed_index
          items = items[:index+1] + [insert_map[key]] + items[index+1:]
          keys = keys[:index+1] + [key] + keys[index+1:]
      else:
          # If the level is not found, append the insert_map to the end of the list
          logger.debug(f"not found, insert it at the end")
          items.append(insert_map[key])
          keys.append(key)
  return items, keys

def insert_additional_parameters_before(items, keys, insert_map):
    """
    Inserts additional parameters into the `items` and `keys` lists before the first occurrence of a matching key.
    Args:
      items (list): The list of items to insert the new parameters into.
      keys (list): The list of keys corresponding to the items.
      insert_map (dict): A dictionary where keys are hierarchical keys (dot-separated) and values are the items to insert.
    Returns:
      tuple: A tuple containing the updated `items` and `keys` lists.
    Example:
      items = ['item1', 'item2']
      keys = ['key1', 'key2']
      insert_map = {'key1.subkey': 'new_item'}
      updated_items, updated_keys = insert_additional_parameters_before(items, keys, insert_map)
    """
    for key, value_to_insert in insert_map.items():

        # Split the key into its hierarchical levels
        levels = key.split('.')
        found = False

        logger.debug(f"Inserting {key} with levels {len(levels)}")
        # Traverse the levels to find the deepest match
        for level_depth in range(len(levels), 0, -1):
            # Reconstruct the level up to the current depth
            level_to_find = '.'.join(levels[:level_depth])

            # Find the index of the first occurrence of the level in keys
            index = next((index for index, item in enumerate(keys) if item.startswith(level_to_find)), None)

            if index is not None:
                # If the level is found, insert the value before it
                items = items[:index] + [value_to_insert] + items[index:]
                keys = keys[:index] + [key] + keys[index:]
                logger.debug(f"with level_depth {level_depth} at position {index}")
                found = True
                break  # Exit the loop once the key is inserted

        if not found:
            # If no match is found, append the value to the end of the lists
            logger.warning(f"parameters_context: {key} not found, insert it at the end of the list")
            items.append(value_to_insert)
            keys.append(key)

    return items, keys


class GeneraterParameterLibraryDetails(SphinxDirective):
    required_arguments = 1
    optional_arguments = 1 # context yaml file, "key: string"

    def run(self):
        # get the absolute path from sphinx tree
        yaml_file = self.env.relfn2path(self.arguments[0], self.env.docname)[0]
        if len(self.arguments) > 1:
          context_yaml_file = self.env.relfn2path(self.arguments[1], self.env.docname)[0]
          with open(context_yaml_file, 'r') as file:
            context_yaml_data = yaml.safe_load(file)
        else:
          context_yaml_data = {}

        gen_param_struct = GenerateCode("rst")
        gen_param_struct.parse(yaml_file, "")

        # general parameters
        param_details = [
            ParameterDetailMarkdown(param)
            for param in gen_param_struct.declare_parameters
        ]
        # runtime parameters, i.e., such with a __map_ key
        runtime_param_details = [
            RuntimeParameterDetailMarkdown(param)
            for param in gen_param_struct.declare_dynamic_parameters
        ]

        param_strings_keys = [detail.declare_parameters.parameter_name for detail in param_details]
        param_items = [str(detail) for detail in param_details]
        runtime_param_strings_map = {detail.declare_parameters.parameter_name: str(detail) for detail in runtime_param_details}
        # add optional context data from yaml. we don't use a jinja template here any more -> add the indent manually
        context_strings_map = {key: str(key) + "\n" +
          '\n'.join('  ' + line for line in str(value).replace('\\t', '  ').splitlines()) + "\n"
          for key, value in context_yaml_data.items()}

        param_items, param_strings_keys = insert_additional_parameters_after(param_items, param_strings_keys, runtime_param_strings_map)
        param_items, param_strings_keys = insert_additional_parameters_before(param_items, param_strings_keys, context_strings_map)

        docs = "\n".join(param_items)

        # logger.info(docs)

        # Add the content one line at a time.
        # Second argument is the filename to report in any warnings
        # or errors, third argument is the line number.
        # rst.append(docs, yaml_file, 10)
        rst = ViewList()
        for line in docs.splitlines():
          rst.append(line, yaml_file)

        node = nodes.section()
        # necessary so that the child nodes get the right source/line set
        node.document = self.state.document
        nested_parse_with_titles(self.state, rst, node)

        return node.children

class GeneraterParameterLibraryDefaultConfig(SphinxDirective):
    required_arguments = 1
    optional_arguments = 0

    def run(self):
        # get the absolute path from sphinx tree
        yaml_file = self.env.relfn2path(self.arguments[0], self.env.docname)[0]

        gen_param_struct = GenerateCode("rst")
        gen_param_struct.parse(yaml_file, "")
        auto_doc = DefaultConfigMarkdown(gen_param_struct)
        docs = str(auto_doc)
        # print(docs)

        # Add the content one line at a time.
        # Second argument is the filename to report in any warnings
        # or errors, third argument is the line number.
        # rst.append(docs, yaml_file, 10)
        rst = ViewList()
        for line in docs.splitlines():
          rst.append(line, yaml_file)

        node = nodes.section()
        # necessary so that the child nodes get the right source/line set
        node.document = self.state.document
        nested_parse_with_titles(self.state, rst, node)

        return node.children

def setup(app):
    app.add_directive("generate_parameter_library_details", GeneraterParameterLibraryDetails)
    app.add_directive("generate_parameter_library_default", GeneraterParameterLibraryDefaultConfig)
    logger.debug("extension generate_parameter_library has been loaded!")

    return {
        'version': '0.2',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
