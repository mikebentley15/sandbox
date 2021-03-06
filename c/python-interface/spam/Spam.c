#include "Spam.h"

#include <stdio.h>

PyMemberDef Spam_members[] = {
  {"brand", T_STRING, offsetof(Spam, brand), 0, "The brand of spam"},
  {NULL}  // Sentinel
};

PyMethodDef Spam_methods[] = {
  {"print", (PyCFunction)Spam_print, METH_NOARGS, "Print the brand"},
  {NULL}  // Sentinal
};

PyTypeObject SpamType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "spam.Spam",               /*tp_name*/
    sizeof(Spam),              /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    (destructor)Spam_dealloc,  /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_reserved */
    0,                         /*tp_repr*/
    0,                         /*tp_as_number*/
    0,                         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,/*tp_flags*/
    "Spam object with brand",  /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Spam_methods,              /* tp_methods */
    Spam_members,              /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Spam_init,       /* tp_init */
    0,                         /* tp_alloc */
    Spam_new,                  /* tp_new */
};

void Spam_dealloc(Spam* self) {
  // free the internal memory first
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Spam_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
  Spam *self;
  self = (Spam*)type->tp_alloc(type, 0);
  if (self != NULL) {
    self->brand = NULL;
  }
  return (PyObject*)self;
}

int Spam_init(Spam *self, PyObject *args, PyObject *kwds) {
  if (!PyArg_ParseTuple(args, "s", &self->brand)) {
    // TODO: do we raise an exception here?
    return -1;
  }
  return 0;
}

PyObject* Spam_print(Spam* self) {
  puts(self->brand);
  Py_RETURN_NONE;
}

