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
	@echo ""
	@echo "make publish"
	@echo "   publish generated html to thesofproject.github.io site:"
	@echo "   specify RELEASE=name to publish as a tagged release version"
	@echo "   and placed in a version subfolder.  Requires repo merge permission."

html-with-api: Makefile
	@echo Single html file with API
	@echo Step 1: Init submodules
	git submodule update --init --recursive 
	@echo Step 2: Creating html files
	$(SPHINXBUILD) $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo Step 3: Building API
	if cd doc/ros2_control; then git checkout master && git pull; else echo "Submodules are not initialized correctly. Exiting!" && exit; fi && \
		doxygen doc/Doxyfile && mkdir -p ../../$(BUILDDIR)/html/doc/api/ && cp -r doc/_build/html/. ../../$(BUILDDIR)/html/doc/api/ &&  \
		rm -rf doc/_build && cd ../../ 

multiversion: Makefile
	@echo Building multi version documentation without API
	@echo Step 1: Creating temporary deployment branches
	./create_deployment_branches
	@echo Step 2: Build multi version documentation
	sphinx-multiversion $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=rolling_deploy/index.html\" /></head></html>" > "$(BUILDDIR)"/html/index.html
	./delet_deployment_branches

multiversion-with-api: Makefile
	@echo Building multi version documentation with API
	@echo Step 1: Creating temporary deployment branches
	./create_deployment_branches
	@echo Step 2: Build multi version documentation
	sphinx-multiversion $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=rolling_deploy/index.html\" /></head></html>" > "$(BUILDDIR)"/html/index.html
	@echo Step 3: Deleting temporary deployment branches
	./delet_deployment_branches
	@echo Step 4: Building API for foxy
	if cd doc/ros2_control; then git checkout foxy && git pull; else echo "Submodules are not initialized correctly. Exiting!" && exit; fi && \
	doxygen doc/Doxyfile && mkdir -p ../../$(BUILDDIR)/html/foxy_deploy/doc/api/ && cp -r doc/_build/html/. ../../$(BUILDDIR)/html/foxy_deploy/doc/api/ &&  \
	rm -rf doc/_build && cd ../../ 
	@echo Step 5: Building API for galactic
	if cd doc/ros2_control; then git checkout galactic && git pull; else echo "Submodules are not initialized correctly. Exiting!" && exit; fi && \
	doxygen doc/Doxyfile && mkdir -p ../../$(BUILDDIR)/html/galactic_deploy/doc/api/ && cp -r doc/_build/html/. ../../$(BUILDDIR)/html/galactic_deploy/doc/api/ &&  \
	rm -rf doc/_build && cd ../../ 
	@echo Step 6: Building API for rolling
	if cd doc/ros2_control; then git checkout rolling && git pull; else echo "Submodules are not initialized correctly. Exiting!" && exit; fi && \
	doxygen doc/Doxyfile && mkdir -p ../../$(BUILDDIR)/html/rolling_deploy/doc/api/ && cp -r doc/_build/html/. ../../$(BUILDDIR)/html/rolling_deploy/doc/api/ &&  \
	rm -rf doc/_build && cd ../../ 

.PHONY: help Makefile html-with-api multiversion html-with-api

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
