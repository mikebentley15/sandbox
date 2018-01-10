#ifndef SPAMFUNCTIONS_H
#define SPAMFUNCTIONS_H

#include <Python.h>

PyObject* spam_system(PyObject *self, PyObject *args);
PyObject* spam_check_system(PyObject *self, PyObject *args);

extern PyObject *SpamError;

#endif // SPAMFUNCTIONS_H
