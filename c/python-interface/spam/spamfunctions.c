#include "spamfunctions.h"

#include <stdlib.h>  // for system()

PyObject *SpamError;

PyObject* spam_system(PyObject *self, PyObject *args) {
  const char *command;
  int status;

  if (!PyArg_ParseTuple(args, "s", &command)) {
    return NULL;
  }
  status = system(command);
  return PyLong_FromLong(status);
}

//
// Version using SpamError
//

PyObject* spam_check_system(PyObject *self, PyObject *args) {
  const char *command;
  int status;

  if (!PyArg_ParseTuple(args, "s", &command)) {
    return NULL;
  }
  status = system(command);
  if (status != 0) {
    PyErr_SetString(SpamError, "System command failed");
    return NULL;
  }
  return PyLong_FromLong(status);
}


