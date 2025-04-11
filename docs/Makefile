# Variables
SPHINXBUILD   = sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

.PHONY: clean html help

help:
	@echo "Please use 'make <target>' where <target> is one of:"
	@echo "  html      to build HTML documentation"
	@echo "  clean     to remove generated files"

html:
	$(SPHINXBUILD) -b html $(SOURCEDIR) $(BUILDDIR)/html
	@echo
	@echo "Build finished. Open $(BUILDDIR)/html/index.html in your browser."

clean:
	rm -rf $(BUILDDIR)/*

