# Minimal makefile for Sphinx documentation
# Author: Denis Stogl (Stogl Robotics Consulting)
# The file is inspired by the Makefile for the navigation.ros.org <https://github.com/ros-planning/navigation.ros.org>

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = python3 -m sphinx
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help $(SOURCEDIR) $(BUILDDIR) $(SPHINXOPTS) $(O)
	@echo "  html-with-api"
	@echo "  html-with-errors"
	@echo "  html-all-subrepos"
	@echo "  html-all-subrepos-with-errors"
	@echo "  html-all-subrepos-with-api"
	@echo "  multiversion"
	@echo "  multiversion-with-api"
	@echo "  multiversion-with-errors"
	@echo "  linkcheck-all-subrepos-with-api"

html-with-errors: Makefile
	@echo Single version without API, no checkout of sub_repos
	$(SPHINXBUILD) $(SPHINXOPTS) -W --keep-going $(SOURCEDIR) $(BUILDDIR)/html

html-with-api: Makefile
	@echo Single version with API
	@echo Step 1: Creating html files
	$(SPHINXBUILD) $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 2: Building API
	./make_help_scripts/create_api.py

html-all-subrepos: Makefile
	@echo Single html file without API
	@echo Step 1: Cloning all subrepositories
	./make_help_scripts/add_sub_repos.py
	@echo Step 2: Building documentation
	$(SPHINXBUILD) $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Deleting subrepositories in doc/ folder
	./make_help_scripts/delete_sub_repos.py

html-all-subrepos-with-errors: Makefile
	@echo Single html file without API
	@echo Step 1: Cloning all subrepositories
	./make_help_scripts/add_sub_repos.py
	@echo Step 2: Building documentation
	$(SPHINXBUILD) $(SPHINXOPTS) -W --keep-going $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Deleting subrepositories in doc/ folder
	./make_help_scripts/delete_sub_repos.py

html-all-subrepos-with-api: Makefile
	@echo Single html file with API
	@echo Step 1: Cloning all subrepositories
	./make_help_scripts/add_sub_repos.py
	@echo Step 2: Building documentation
	$(SPHINXBUILD) $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Building API
	./make_help_scripts/create_api.py
	@echo Step 4: Deleting subrepositories in doc/ folder
	./make_help_scripts/delete_sub_repos.py

linkcheck-all-subrepos-with-api: Makefile
	@echo Single version html file with API and linkcheck
	@echo Step 1: Cloning all subrepositories
	./make_help_scripts/add_sub_repos.py
	@echo Step 2: Building API
	./make_help_scripts/create_api.py
	@echo Step 3: Check links
	./make_help_scripts/check_links.py $(BUILDDIR)/html
	@echo Step 4: Deleting subrepositories in doc/ folder
	./make_help_scripts/delete_sub_repos.py

multiversion: Makefile
	@echo Building multi version documentation without API
	@echo Step 1: Creating temporary commits
	./make_help_scripts/add_tmp_commits.py
	@echo Step 2: Build multi version documentation
	sphinx-multiversion $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Deleting temporary commits
	./make_help_scripts/delete_tmp_commits.py
	@echo Step 4: Create correct index + legacy master version
	./make_help_scripts/fix_index.py --builddir $(BUILDDIR)

multiversion-with-errors: Makefile
	@echo Building multi version documentation without API
	@echo Step 1: Creating temporary commits
	./make_help_scripts/add_tmp_commits.py
	@echo Step 2: Build multi version documentation
	sphinx-multiversion $(SPHINXOPTS) -W --keep-going $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Deleting temporary commits
	./make_help_scripts/delete_tmp_commits.py
	@echo Step 4: Create correct index + legacy master version
	./make_help_scripts/fix_index.py --builddir $(BUILDDIR)

multiversion-with-api: Makefile
	@echo Building multi version documentation with API
	@echo Step 1: Creating temporary commits
	./make_help_scripts/add_tmp_commits.py
	@echo Step 2: Build multi version documentation
	sphinx-multiversion $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Deleting temporary commits
	./make_help_scripts/delete_tmp_commits.py
	@echo Step 4: Cleanup subrepos, just to be sure
	./make_help_scripts/delete_sub_repos.py
	@echo Step 5: Building multiversion API
	./make_help_scripts/create_api_multi_version.py
	@echo Step 6: Create correct index + legacy master version
	./make_help_scripts/fix_index.py --builddir $(BUILDDIR)

.PHONY: help Makefile html-with-errors html-with-api multiversion multiversion-with-api multiversion-with-errors html-all-subrepos html-all-subrepos-with-api html-all-subrepos-with-errors linkcheck-all-subrepos-with-api

# TODO(denis): Enable this!
# # # # Generate the doxygen xml (for Sphinx) and copy the doxygen html to the
# # # # api folder for publishing along with the Sphinx-generated API docs.
# # #
# # # html:
# # # 	$(Q)$(SPHINXBUILD) -t $(DOC_TAG) -b html -d $(BUILDDIR)/doctrees $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS) $(O)

# Remove generated content (Sphinx and doxygen)
# only remove html and doctrees directories since pip also uses _build for side-packages
clean:
	rm -fr $(BUILDDIR)/html $(BUILDDIR)/doctrees

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ $(SOURCEDIR) $(BUILDDIR) $(SPHINXOPTS) $(O)
