from docutils import nodes
from sphinx.util.docutils import SphinxDirective
from sphinx.util.nodes import nested_parse_with_titles
from docutils.statemachine import ViewList
import yaml
import re
from generate_parameter_library_py.parse_yaml import (
    GenerateCode,
)
from generate_parameter_library_py.generate_markdown import (
    DefaultConfigMarkdown,
    ParameterDetailMarkdown,
    RuntimeParameterDetailMarkdown
)

def insert_additional_parameters_after(items, keys, insert_map):
  for key in insert_map.keys():
      # Define the level you're looking for
      level_to_find = key.split('.')[0]

      # Reverse keys and find the index of the first occurrence of the level
      reversed_keys = list(reversed(keys))
      reversed_index = next((index for (index, item) in enumerate(reversed_keys) if item.split('.')[0] == level_to_find), None)

      if reversed_index is not None:
          # If the level is found, insert the runtime_param_item after it in the original list
          index = len(keys) - 1 - reversed_index
          items = items[:index+1] + [insert_map[key]] + items[index+1:]
          keys = keys[:index+1] + [key] + keys[index+1:]
      else:
          # If the level is not found, append the runtime_param_item to the end of the list
          items.append(insert_map[key])
          keys.append(key)
  return items, keys

def insert_additional_parameters_before(items, keys, insert_map):
  for key in insert_map.keys():
      # Define the level you're looking for
      level_to_find = key.split('.')[0]

      # Find the index of the first occurrence of the level in items
      index = next((index for (index, item) in enumerate(keys) if item.split('.')[0] == level_to_find), None)

      if index is not None:
          # If the level is found, insert the runtime_param_item after it
          items = items[:index] + [insert_map[key]] + items[index:]
          keys = keys[:index] + [key] + keys[index:]
      else:
          # If the level is not found, append the runtime_param_item to the end of the list
          items.append(insert_map[key])
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

        param_strings_keys = [detail.declare_parameters.parameter_name  for detail in param_details]
        param_items = [str(detail) for detail in param_details]
        runtime_param_strings_map = {detail.declare_parameters.parameter_name: str(detail) for detail in runtime_param_details}
        # add optional context data from yaml. we don't use a jinja template here -> add the indent manually
        context_strings_map = {key: str(key) + "\n" +
          '\n'.join('  ' + line for line in str(value).replace('\\t', '  ').splitlines()) + "\n"
          for key, value in context_yaml_data.items()}

        param_items, param_strings_keys = insert_additional_parameters_after(param_items, param_strings_keys, runtime_param_strings_map)
        param_items, param_strings_keys = insert_additional_parameters_before(param_items, param_strings_keys, context_strings_map)

        docs = "\n".join(param_items)

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

    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
