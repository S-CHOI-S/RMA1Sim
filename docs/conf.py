import subprocess, os

extensions = [
    "sphinx_rtd_theme",
    "breathe",
    'sphinx.ext.autosectionlabel'
]

html_theme = "sphinx_rtd_theme"

# General information about the project.
project = 'raisim'
copyright = '2020, RaiSim Tech Inc.'
author = 'Yeonjoo Chung and Jemin Hwangbo'
version = '1.0.0'
release = '1.0.0'
# Output file base name for HTML help builder.
htmlhelp_basename = 'raisim_doc'
html_show_sourcelink = False
# Breathe Configuration
breathe_default_project = "raisim"
autosectionlabel_prefix_document = True
autosectionlabel_maxdepth = 4