from docutils import nodes
from sphinx.util.docutils import SphinxDirective
from sphinx.util.nodes import nested_parse_with_titles
from docutils.statemachine import ViewList
from generate_parameter_library_py.parse_yaml import (
    GenerateCode,
)
from generate_parameter_library_py.generate_markdown import (
    DefaultConfigMarkdown,
    ParameterDetailMarkdown
)

class GeneraterParameterLibraryDetails(SphinxDirective):
    required_arguments = 1
    optional_arguments = 0

    def run(self):
        # get the absolute path from sphinx tree
        yaml_file = self.env.relfn2path(self.arguments[0], self.env.docname)[0]

        gen_param_struct = GenerateCode("rst")
        gen_param_struct.parse(yaml_file, "")

        param_details = [
            ParameterDetailMarkdown(param)
            for param in gen_param_struct.declare_parameters
        ]
        docs = "\n".join(str(val) for val in param_details)
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
