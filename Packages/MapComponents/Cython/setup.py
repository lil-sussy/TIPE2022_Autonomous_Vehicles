import Cython.Build as cy
import distutils.core as util
import os
import sys
import numpy # Dependencies

name = "EuclidianCurves"
path = "EuclidianCurves.pyx"

# define an extension that will be cythonized and compiled
ext = util.Extension(name=name, sources=[os.path.realpath(path)], include_dirs=[numpy.get_include()])
app = cy.cythonize(ext)
util.setup(ext_modules = app)