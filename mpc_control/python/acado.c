// Python interface for ACADO generated code
// see setup.py for for installation

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <Python.h>
#include <numpy/arrayobject.h>        
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdio.h>


#if PY_MAJOR_VERSION >= 3
#define PY3K
#endif


#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */


/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
static int initialized = 0;
static int verbose = 0;
static int iter = 0;


int getMatrix(PyArrayObject *src, real_t* dest, int rows, int cols, char *desc){
  if ((PyArray_DIM(src, 0) != rows) || (PyArray_DIM(src, 1) != cols))
  {  
    PyErr_Format(PyExc_ValueError, "Expecting double matrix %dx%d for %s", rows, cols, desc);
    return 0;
  } 
  if (verbose == 2) PySys_WriteStdout("%s:\n", desc);
  for (int i=0; i < rows; i++)
  {
    for (int j=0; j < cols; j++){
      double elem=*((double *)PyArray_GETPTR2(src,i,j));
      dest[i * cols + j] = elem;
      if (verbose == 2) PySys_WriteStdout("%.3f ", elem);
    }
    if (verbose == 2) PySys_WriteStdout("\n");
  }
  if (verbose == 2) PySys_WriteStdout("\n");
  return 1; 
};

void setMatrix(real_t *src, PyArrayObject **dest, int rows, int cols){ 
  npy_intp dims[] = {rows,cols};
  *dest = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
  //*dest = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_DOUBLE);
  double *p = (double*)PyArray_DATA(*dest);
  for (int i=0; i < rows; i++)
  {
    for (int j=0; j < cols; j++){
      p[i * cols + j] = src[i * cols + j];
    }
  } 
}

// Python MPC module function
// acado.mpc(forceInit, doFeedback, x0, X, U, Y, yN, W, WN, verbose)
//   forceInit: force solver initialization? (0/1)
//   doFeedback: do feedback step? (0/1)
//   x0: initial state feedback
//   X: differential variable vectors.
//   U: control variable vectors.
//   Y: reference/measurement vectors of first N nodes.
//   yN: Reference/measurement vector for N+1 node.
//   W: weight matrix
//   WN: weight matrix
//   verbose: verbosity level (0-2)
static PyObject* mpc(PyObject *self, PyObject *args)
{
    PyArrayObject *x0;
    PyArrayObject *x;
    PyArrayObject *u;
    PyArrayObject *y;
    PyArrayObject *yN;
    PyArrayObject *W;
    PyArrayObject *WN;
    int forceInit;
    int doFeedback;
    acado_timer t;
    if (!PyArg_ParseTuple(args, "iiOOOOOOOi", &forceInit, &doFeedback, &x0, &x, &u, &y, &yN, &W, &WN, &verbose)){
      return NULL;
    } 
    if (!getMatrix(x0, acadoVariables.x0, 1,   NX, "x0")) return NULL;
    if (!getMatrix(x, acadoVariables.x,   N+1, NX, "X")) return NULL;
    if (!getMatrix(u, acadoVariables.u,   N,   NU, "U")) return NULL;
    if (!getMatrix(y, acadoVariables.y,   N,   NY, "Y")) return NULL;
    if (!getMatrix(yN, acadoVariables.yN, 1,   NYN,"yN")) return NULL;
    if (!getMatrix(W, acadoVariables.W,   NY*N,NY, "W")) return NULL;
    if (!getMatrix(WN, acadoVariables.WN, NYN, NYN,"WN")) return NULL;
   
    if( verbose == 2 ) acado_printHeader();
    acado_tic( &t );
    if ((!initialized) || (forceInit)){
      if (verbose == 2) PySys_WriteStdout("acado_initializeSolver\n");
      acado_initializeSolver();       
      initialized = 1;
      iter = 0;
    } else {
      if (verbose == 2) PySys_WriteStdout("acado_shiftStates, acado_shiftControls()\n");
      // acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
      acado_shiftStates(2, 0, 0);  
      // acado_shiftControls( real_t* const uEnd )
      acado_shiftControls( 0 ); 
    }
    if (verbose == 2) PySys_WriteStdout("acado_preparationStep()\n");
    acado_preparationStep();
    if (doFeedback) {     
      if (verbose == 2) PySys_WriteStdout("acado_acado_feedbackStep()\n");
      acado_feedbackStep();
    }
    real_t te = acado_toc( &t );
    if( verbose == 1)  {
      PySys_WriteStdout("\tReal-Time Iteration %d:  KKT Tolerance = %.3e  duration = %.3g microseconds\n", iter, acado_getKKT(), 1e6 * te );
    }
    if( verbose == 2)  {  
      acado_printDifferentialVariables();
      acado_printControlVariables();    
    }
    iter++;
    setMatrix(acadoVariables.u, &u, N,   NU);
    setMatrix(acadoVariables.x, &x, N+1, NX);
    PyObject* ret = Py_BuildValue("OO", x, u);
    Py_XDECREF(u);
    Py_XDECREF(x);
    return ret;
};

// registration table
static PyMethodDef acado_methods[]={
    {"mpc", mpc, METH_VARARGS, "func doc"},
    {NULL, NULL, 0, NULL}
};

#ifdef PY3K
// ----------- module definition structure for python3 --------------
static struct PyModuleDef acado_module = {
    PyModuleDef_HEAD_INIT,
    "acado",
    "mod doc",
    -1,
    acado_methods
};
// module initializer for python3
PyMODINIT_FUNC PyInit_acado()
{
  import_array();  
  return PyModule_Create(&acado_module);
};
#else  
// ---------- module initializer for python2 --------------------------
PyMODINIT_FUNC initacado() {
    Py_InitModule3("acado", acado_methods, "mod doc");
    import_array();
}
#endif



