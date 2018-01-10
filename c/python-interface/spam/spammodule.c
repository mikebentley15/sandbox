#include <Python.h>

#include "spamfunctions.h"
#include "Spam.h"
#include "Noddy.h"

static PyMethodDef SpamMethods[] = {
  {"system", spam_system, METH_VARARGS,
    "Execute a shell command.  Returns the status."},
  {"check_system", spam_check_system, METH_VARARGS,
    "Execute a shell command.  Raises a SpamError if status is not 0."},
  { NULL } // Sentinel
};

static struct PyModuleDef spammodule = {
  PyModuleDef_HEAD_INIT,
  "spam",   /* name of module */
  // module documentation
  "spam module from the example in the docs",
  -1,       /* size of per-interpreter state of the module,
               or -1 if the module keeps state in global variables. */
  SpamMethods
};

PyMODINIT_FUNC PyInit_spam(void) {
  PyObject *m;

  if (PyType_Ready(&SpamType) < 0) return NULL;

  Noddy1Type.tp_new = PyType_GenericNew;
  if (PyType_Ready(&Noddy1Type) < 0) return NULL;
  if (PyType_Ready(&Noddy2Type) < 0) return NULL;
  if (PyType_Ready(&Noddy3Type) < 0) return NULL;
  if (PyType_Ready(&Noddy4Type) < 0) return NULL;

  m = PyModule_Create(&spammodule);
  if (m == NULL)
    return NULL;

  SpamError = PyErr_NewException("spam.SpamError", NULL, NULL);
  Py_INCREF(SpamError);
  PyModule_AddObject(m, "SpamError", SpamError);
  Py_INCREF(&SpamType);
  PyModule_AddObject(m, "Spam", (PyObject*)&SpamType);
  Py_INCREF(&Noddy1Type);
  PyModule_AddObject(m, "Noddy1", (PyObject*)&Noddy1Type);
  Py_INCREF(&Noddy2Type);
  PyModule_AddObject(m, "Noddy2", (PyObject*)&Noddy2Type);
  Py_INCREF(&Noddy3Type);
  PyModule_AddObject(m, "Noddy3", (PyObject*)&Noddy3Type);
  Py_INCREF(&Noddy4Type);
  PyModule_AddObject(m, "Noddy4", (PyObject*)&Noddy4Type);
  return m;
}

