#ifndef SPAM_H
#define SPAM_H

#include <Python.h>
#include <structmember.h>

typedef struct {
  PyObject_HEAD
  char* brand;
} Spam;

extern PyMemberDef Spam_members[];
extern PyMethodDef Spam_methods[];
extern PyTypeObject SpamType;

void Spam_dealloc(Spam* self);
PyObject* Spam_new(PyTypeObject *type, PyObject *args, PyObject *kwds);
int Spam_init(Spam *self, PyObject *args, PyObject *kwds);
PyObject* Spam_print(Spam* self);

#endif // SPAM_H
