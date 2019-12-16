#
# install with:
#   python3 setup.py build install --force
#
# Anaconda (Windows):
# 1.  conda install m2w64-toolchain
#     conda install -c anaconda libpython
#     conda install -c msys2 m2w64-toolchain
# 2.  echo [build] > %CONDA_PREFIX%\Lib\distutils\distutils.cfg
#     echo compiler = mingw32 >> %CONDA_PREFIX%\Lib\distutils\distutils.cfg
# 3. install (see above) will generate d:\Users\...\Miniconda3\envs\py36\Lib\site-packages\acado.cp36-win_amd64.pyd

import numpy
from distutils.core import setup, Extension
setup(name = 'acado', version = '1.0',  \
   ext_modules = [   \
       Extension('acado', \
         ['acado.c',  \
          'acado_auxiliary_functions.c', \
          'acado_integrator.c',  \
          'acado_qpoases_interface.cpp', \
          'acado_solver.c', \
    	  'qpoases/SRC/Bounds.cpp', \
    	  'qpoases/SRC/Constraints.cpp', \
    	  'qpoases/SRC/CyclingManager.cpp', \
    	  'qpoases/SRC/Indexlist.cpp',  \
    	  'qpoases/SRC/MessageHandling.cpp', \
    	  'qpoases/SRC/QProblem.cpp', \
    	  'qpoases/SRC/QProblemB.cpp', \
    	  'qpoases/SRC/SubjectTo.cpp', \
    	  'qpoases/SRC/Utils.cpp', \
    	  'qpoases/SRC/EXTRAS/SolutionAnalysis.cpp', \
         ],
         include_dirs=['.', numpy.get_include(), 'qpoases', 'qpoases/INCLUDE', 'qpoases/SRC']
         )  \
   ]  \
)
