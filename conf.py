# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import itertools
import os
import sys
import time

from docutils.parsers.rst import Directive

sys.path.append(os.path.abspath("./sphinx-multiversion"))

# -- General configuration -------------------------------------------------
# General information about the project.
project = "control.ros.org"
author = "ros2_control Development Team"
copyright = "{}, {}".format(time.strftime("%Y"), author)

# Adjust those to change ros distribution
# you might also need to white list branch
ros_distro = "rolling"
distro_title = "Rolling"
distro_title_full = "Rolling Ridley"
repos_file_branch = "master"

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version
version = ""
# The full version, including alpha/beta/rc tags
release = "{}".format(time.strftime("%b %Y"))

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = "1.0"

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# The suffix(es) of source filenames.
source_suffix = ".rst"

# The master toctree document.
master_doc = "index"

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = "en"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "**/CHANGELOG.rst", "**/README.rst"]

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "sphinx"

# -- Extension configuration -------------------------------------------------
# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named "sphinx.ext.*") or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx.ext.coverage",
    "sphinx.ext.githubpages",
    "sphinx_rtd_theme",
    "sphinx_multiversion",
    "sphinx.ext.extlinks",
    "sphinx.ext.ifconfig",
    "sphinx_copybutton",
    "sphinx.ext.viewcode",
    "sphinxcontrib.globalsubs"
]

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True

# -- Options for HTML output -------------------------------------------------

html_baseurl = "https://control.ros.org/" + ros_distro + "/"
# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme_path = ["_themes"]
html_theme = "sphinx_rtd_theme"

templates_path = [
    "_templates",
]

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ["_static"]

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``["localtoc.html", "relations.html", "sourcelink.html",
# "searchbox.html"]``.
#
# html_sidebars = {}

html_context = {
    "display_github": True,
    "github_user": "ros-controls",
    "github_repo": "control.ros.org",
    "github_version": repos_file_branch + "/",
    "conf_py_path": "/",
    "source_suffix": source_suffix,
    "css_files": ["_static/override.css"],
    "favicon": "favicon_ros-controls.ico",
    "logo": "logo_ros-controls.png"
}

html_favicon = "images/favicon_ros-controls.ico"
html_logo = "images/logo_ros-controls.png"

html_theme_options = {
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": -1,
    # Only display the logo image, do not display the project name at the top of the sidebar
    "logo_only": True,
}

github_url = "https://github.com/ros-controls/control.ros.org"

# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = "ros2ControlDocumentation"


# -- Extension configuration -------------------------------------------------
# copy transmission images to ros2_control/doc/_build/xml/ so that breathe picks them up as artifacts
# shutil.copytree("ros2_control/transmission_interface/images/", "ros2_control/doc/_build/xml/", dirs_exist_ok=True)
# # Copy resources folders
# shutil.copytree("resources/", "_build/html/resources/", dirs_exist_ok=True)
# # generate doxygen documentation
# subprocess.run(["doxygen", "doc/Doxyfile"], cwd="ros2_control")
# # copy doxygen documentation to api subfolder
# shutil.copytree("ros2_control/doc/_build/html", "_build/html/api")


# Drop any source link suffix
html_sourcelink_suffix = ""


# Add branches you want to whtielist here.
smv_branch_whitelist = r"^(foxy|galactic|humble|master)$"
smv_released_pattern = r"^refs/(heads|remotes/[^/]+)/(foxy|galactic|humble).*$"
smv_remote_whitelist = r"^(origin)$"
smv_latest_version = "humble"
smv_eol_versions = []

distro_full_names = {
    "foxy": "Foxy Fitzroy",
    "galactic": "Galactic Geochelone",
    "humble": "Humble Hawksbill",
    "rolling": "Rolling Ridley",
}

# These default values will be overridden when building multiversion
macros = {
    "DISTRO": ros_distro,
    "DISTRO_TITLE": distro_title,
    "DISTRO_TITLE_FULL": distro_title_full,
    "REPOS_FILE_BRANCH": repos_file_branch,
}


# Add any paths that contain custom themes here, relative to this directory.
class RedirectFrom(Directive):
    has_content = True
    template_name = "layout.html"
    redirections = {}

    @classmethod
    def register(cls, app):
        app.connect("html-collect-pages", cls.generate)
        app.add_directive("redirect-from", cls)
        return app

    @classmethod
    def generate(cls, app):
        from sphinx.builders.html import StandaloneHTMLBuilder

        if not isinstance(app.builder, StandaloneHTMLBuilder):
            return

        redirect_html_fragment = """
            <link rel="canonical" href="{base_url}/{url}" />
            <meta http-equiv="refresh" content="0; url={url}" />
            <script>
                window.location.href = '{url}';
            </script>
        """
        redirections = {
            os.path.splitext(os.path.relpath(document_path, app.srcdir))[0]: redirect_urls
            for document_path, redirect_urls in cls.redirections.items()
        }
        redirection_conflict = next(
            (
                (canon_1, canon_2, redirs_1.intersection(redirs_2))
                for (canon_1, redirs_1), (canon_2, redirs_2) in itertools.combinations(
                    redirections.items(), 2
                )
                if redirs_1.intersection(redirs_2)
            ),
            None,
        )
        if redirection_conflict:
            canonical_url_1, canonical_url_2 = redirection_conflict[:2]
            conflicting_redirect_urls = redirection_conflict[-1]
            raise RuntimeError(
                "Documents {} and {} define conflicting redirects: {}".format(
                    canonical_url_1, canonical_url_2, conflicting_redirect_urls
                )
            )
        all_canonical_urls = set(redirections.keys())
        all_redirect_urls = {
            redirect_url
            for redirect_urls in redirections.values()
            for redirect_url in redirect_urls
        }
        conflicting_urls = all_canonical_urls.intersection(all_redirect_urls)
        if conflicting_urls:
            raise RuntimeError(
                f"Some redirects conflict with existing documents: {conflicting_urls}"
            )

        for canonical_url, redirect_urls in redirections.items():
            for redirect_url in redirect_urls:
                context = {
                    "canonical_url": os.path.relpath(canonical_url, redirect_url),
                    "title": os.path.basename(redirect_url),
                    "metatags": redirect_html_fragment.format(
                        base_url=app.config.html_baseurl,
                        url=app.builder.get_relative_uri(redirect_url, canonical_url),
                    ),
                }
                yield (redirect_url, context, cls.template_name)

    def run(self):
        document_path = self.state.document.current_source
        if document_path not in RedirectFrom.redirections:
            RedirectFrom.redirections[document_path] = set()
        RedirectFrom.redirections[document_path].update(self.content)
        return []


def make_router(origin, destination):
    def _missing_reference(app, env, node, contnode):
        from docutils import nodes
        from sphinx.util import docname_join

        doctarget = docname_join(node["refdoc"], node["reftarget"])
        if doctarget.startswith(origin):
            routed_doctarget = doctarget.replace(origin, destination)
            if routed_doctarget in env.all_docs:
                newnode = nodes.reference("", contnode.astext(), internal=True)
                newnode["refuri"] = app.builder.get_relative_uri(node["refdoc"], routed_doctarget)
                return newnode

    return _missing_reference


def smv_rewrite_configs(app, config):
    # When using Sphinx multiversion, there is no way at initial configuration time
    # to determine the distribution we are currently targeting (conf.py is read before
    # external defines are setup, and environment variables aren't passed through to
    # conf.py).  Instead, hook into the 'config-inited' event which is late enough
    # to rewrite the various configuration items with the current version.
    branch_distro = {
        "master": "rolling",
        "humble": "humble",
        "foxy": "foxy",
        "galactic": "galactic"
    }
    if app.config.smv_current_version != "":
        branch = app.config.smv_current_version
        distro = branch_distro[branch]

        # Override default values
        app.config.macros = {
            "DISTRO": distro,
            "DISTRO_TITLE": distro.title(),
            "DISTRO_TITLE_FULL": distro_full_names[distro],
            "REPOS_FILE_BRANCH": branch,
        }
        app.config.html_baseurl = app.config.html_baseurl + "/" + distro + "/"
        app.config.project = "ROS2_Control: " + distro.title()
        app.config.html_logo = "images/logo_ros-controls.png"

        # see https://github.com/missinglinkelectronics/sphinxcontrib-globalsubs
        app.config.global_substitutions = {
            'github_branch': branch,
        }
    else:
        # If we are not building a multiversion build, default to the rolling logo
        app.config.html_logo = "images/logo_ros-controls.png"
        # see https://github.com/missinglinkelectronics/sphinxcontrib-globalsubs
        app.config.global_substitutions = {
            'github_branch': 'master',
        }


def github_link_rewrite_branch(app, pagename, templatename, context, doctree):
    if app.config.smv_current_version != "":
        context["github_version"] = app.config.smv_current_version + "/"
        context["eol_versions"] = app.config.smv_eol_versions


def expand_macros(app, docname, source):
    result = source[0]
    for key, value in app.config.macros.items():
        result = result.replace(f"{{{key}}}", value)
    source[0] = result


def setup(app):
    app.connect("config-inited", smv_rewrite_configs)
    app.connect("html-page-context", github_link_rewrite_branch)
    app.connect("source-read", expand_macros)
    app.add_config_value("smv_eol_versions", [], "html")
    app.add_config_value("macros", {}, True)
    RedirectFrom.register(app)
