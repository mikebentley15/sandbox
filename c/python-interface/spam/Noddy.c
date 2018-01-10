#include "Noddy.h"

#include <stdio.h>

//
// Noddy1: a simple object with nothing
//

PyTypeObject Noddy1Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "spam.Noddy1",             /* tp_name */
    sizeof(Noddy1),            /* tp_basicsize */
    0,                         /* tp_itemsize */
    0,                         /* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_reserved */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash  */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,        /* tp_flags */
    "Noddy objects",           /* tp_doc */
};

//
// Noddy2: Members (first, last, number)
//

PyMemberDef Noddy2_members[] = {
  {"first", T_OBJECT_EX, offsetof(Noddy2, first), 0, "first name"},
  {"last", T_OBJECT_EX, offsetof(Noddy2, last), 0, "last name"},
  {"number", T_INT, offsetof(Noddy2, number), 0, "noddy2 number"},
  {NULL}  // Sentinel
};

void Noddy2_dealloc(Noddy2* self) {
  Py_XDECREF(self->first);
  Py_XDECREF(self->last);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Noddy2_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
  Noddy2 *self;

  self = (Noddy2*)type->tp_alloc(type, 0);
  if (self != NULL) {
    self->first = PyUnicode_FromString("");
    if (self->first == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->last = PyUnicode_FromString("");
    if (self->last == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->number = 0;
  }

  return (PyObject*)self;
}

int Noddy2_init(Noddy2 *self, PyObject *args, PyObject *kwds) {
  PyObject *first = NULL, *last = NULL, *tmp;
  static char *kwlist[] = {"first", "last", "number", NULL};

  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOi", kwlist, &first, &last,
                                   &self->number))
  {
    return -1;
  }

  if (first) {
    tmp = self->first;
    Py_INCREF(first);
    self->first = first;
    Py_XDECREF(tmp);
  }

  if (last) {
    tmp = self->last;
    Py_INCREF(last);
    self->last = last;
    Py_XDECREF(tmp);
  }

  return 0;
}

PyObject* Noddy2_name(Noddy2* self) {
  if (self->first == NULL) {
    PyErr_SetString(PyExc_AttributeError, "first");
    return NULL;
  }

  if (self->last == NULL) {
    PyErr_SetString(PyExc_AttributeError, "last");
    return NULL;
  }

  return PyUnicode_FromFormat("%S %S", self->first, self->last);
}

PyMethodDef Noddy2_methods[] = {
  {"name", (PyCFunction)Noddy2_name, METH_NOARGS,
    "Return the name, combining the first and last names"},
  {NULL}  // Sentinel
};

PyTypeObject Noddy2Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "noddy.Noddy2",            /* tp_name */
    sizeof(Noddy2),            /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)Noddy2_dealloc, /* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_reserved */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash  */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT |
        Py_TPFLAGS_BASETYPE,   /* tp_flags */
    "Noddy2 implementation",   /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Noddy2_methods,            /* tp_methods */
    Noddy2_members,            /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Noddy2_init,     /* tp_init */
    0,                         /* tp_alloc */
    Noddy2_new,                /* tp_new */
};

//
// Noddy3: getter + setter, restricting first and last to string
//

PyMemberDef Noddy3_members[] = {
  {"number", T_INT, offsetof(Noddy3, number), 0, "Noddy3 number"},
  {NULL}  // Sentinel
};

PyGetSetDef Noddy3_getsetters[] = {
  {"first", (getter)Noddy3_getfirst, (setter)Noddy3_setfirst, "first name",
   NULL},
  {"last", (getter)Noddy3_getlast, (setter)Noddy3_setlast, "last name", NULL},
  {NULL}  // Sentinel
};

PyMethodDef Noddy3_methods[] = {
  {"name", (PyCFunction)Noddy3_name, METH_NOARGS,
   "Return the name, combining the first and last name"},
  {NULL}  // Sentinel
};

PyTypeObject Noddy3Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "noddy.Noddy3",            /* tp_name */
    sizeof(Noddy3),            /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)Noddy3_dealloc,/* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_reserved */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash  */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT |
        Py_TPFLAGS_BASETYPE,   /* tp_flags */
    "Noddy3 implementation",   /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Noddy3_methods,            /* tp_methods */
    Noddy3_members,            /* tp_members */
    Noddy3_getsetters,         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Noddy3_init,     /* tp_init */
    0,                         /* tp_alloc */
    Noddy3_new,                /* tp_new */
};

void Noddy3_dealloc(Noddy3* self) {
  Py_XDECREF(self->first);
  Py_XDECREF(self->last);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Noddy3_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  Noddy3 *self;

  self = (Noddy3*)type->tp_alloc(type, 0);
  if (self != NULL) {
    self->first = PyUnicode_FromString("");
    if (self->first == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->last = PyUnicode_FromString("");
    if (self->last == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->number = 0;
  }

  return (PyObject*)self;
}

int Noddy3_init(Noddy3* self, PyObject* args, PyObject* kwds) {
  PyObject *first = NULL, *last = NULL; //, *tmp;

  static char *kwlist[] = {"first", "last", "number", NULL};

  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOi", kwlist,
                                   &first, &last, &self->number))
  {
    return -1;
  }

  if (first) if (0 > Noddy3_setfirst(self, first, NULL)) return -1;
  //if (first) {
  //  tmp = self->first;
  //  Py_INCREF(first);
  //  self->first = first;
  //  Py_DECREF(tmp);
  //}

  if (last) if (0 > Noddy3_setlast(self, last, NULL)) return -1;
  //if (last) {
  //  tmp = self->last;
  //  Py_INCREF(last);
  //  self->last = last;
  //  Py_DECREF(tmp);
  //}

  return 0;
}

PyObject* Noddy3_getfirst(Noddy3* self, void* closure) {
  Py_INCREF(self->first);
  return self->first;
}

int Noddy3_setfirst(Noddy3* self, PyObject* value, void* closure) {
  if (value == NULL) {
    PyErr_SetString(PyExc_TypeError, "Cannot delete the first attribute");
    return -1;
  }

  if (!PyUnicode_Check(value)) {
    PyErr_SetString(PyExc_TypeError,
                    "The first attribute value must be a string");
    return -1;
  }

  PyObject* tmp = self->first;
  Py_INCREF(value);
  self->first = value;
  Py_DECREF(tmp);

  return 0;
}

PyObject* Noddy3_getlast(Noddy3* self, void* closure) {
  Py_INCREF(self->last);
  return self->last;
}

int Noddy3_setlast(Noddy3* self, PyObject* value, void* closure) {
  if (value == NULL) {
    PyErr_SetString(PyExc_TypeError, "Cannot delete the last attribute");
    return -1;
  }

  if (!PyUnicode_Check(value)) {
    PyErr_SetString(PyExc_TypeError,
                    "The last attribute value must be a string");
    return -1;
  }

  PyObject* tmp = self->last;
  Py_INCREF(value);
  self->last = value;
  Py_DECREF(tmp);

  return 0;
}

PyObject* Noddy3_name(Noddy3* self) {
  return PyUnicode_FromFormat("%S %S", self->first, self->last);
}

//
// Noddy4: Support cyclic garbage collection, traverse + clear
//

PyMemberDef Noddy4_members[] = {
  {"first", T_OBJECT_EX, offsetof(Noddy4, first), 0, "first name"},
  {"last", T_OBJECT_EX, offsetof(Noddy4, last), 0, "last name"},
  {"number", T_INT, offsetof(Noddy4, number), 0, "Noddy4 number"},
  {NULL}  // Sentinel
};

PyMethodDef Noddy4_methods[] = {
  {"name", (PyCFunction)Noddy4_name, METH_NOARGS,
   "Return the name, combining the first and last names"},
  {NULL}  // Sentinel
};

PyTypeObject Noddy4Type = {
  PyVarObject_HEAD_INIT(NULL, 0)
  "spam.Noddy4",             /* tp_name */
  sizeof(Noddy4),            /* tp_basicsize */
  0,                         /* tp_itemsize */
  (destructor)Noddy4_dealloc,/* tp_dealloc */
  0,                         /* tp_print */
  0,                         /* tp_getattr */
  0,                         /* tp_setattr */
  0,                         /* tp_reserved */
  0,                         /* tp_repr */
  0,                         /* tp_as_number */
  0,                         /* tp_as_sequence */
  0,                         /* tp_as_mapping */
  0,                         /* tp_hash  */
  0,                         /* tp_call */
  0,                         /* tp_str */
  0,                         /* tp_getattro */
  0,                         /* tp_setattro */
  0,                         /* tp_as_buffer */
  Py_TPFLAGS_DEFAULT |
      Py_TPFLAGS_BASETYPE |
      Py_TPFLAGS_HAVE_GC,    /* tp_flags */
  "Noddy4 implementation",   /* tp_doc */
  (traverseproc)Noddy4_traverse, /* tp_traverse */
  (inquiry)Noddy4_clear,     /* tp_clear */
  0,                         /* tp_richcompare */
  0,                         /* tp_weaklistoffset */
  0,                         /* tp_iter */
  0,                         /* tp_iternext */
  Noddy4_methods,            /* tp_methods */
  Noddy4_members,            /* tp_members */
  0,                         /* tp_getset */
  0,                         /* tp_base */
  0,                         /* tp_dict */
  0,                         /* tp_descr_get */
  0,                         /* tp_descr_set */
  0,                         /* tp_dictoffset */
  (initproc)Noddy4_init,     /* tp_init */
  0,                         /* tp_alloc */
  Noddy4_new,                /* tp_new */
};

int Noddy4_traverse(Noddy4* self, visitproc visit, void* arg) {
  Py_VISIT(self->first);
  Py_VISIT(self->last);
  return 0;
}

int Noddy4_clear(Noddy4* self) {
  Py_CLEAR(self->first);
  Py_CLEAR(self->last);
  return 0;
}

void Noddy4_dealloc(Noddy4* self) {
  PyObject_GC_UnTrack(self);
  Noddy4_clear(self);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Noddy4_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  Noddy4 *self;

  self = (Noddy4*)type->tp_alloc(type, 0);
  if (self != NULL) {
    self->first = PyUnicode_FromString("");
    if (self->first == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->last = PyUnicode_FromString("");
    if (self->last == NULL) {
      Py_DECREF(self);
      return NULL;
    }

    self->number = 0;
  }

  return (PyObject*)self;
}

int Noddy4_init(Noddy4* self, PyObject* args, PyObject* kwds) {
  PyObject *first = NULL, *last = NULL, *tmp;
  static char *kwlist[] = {"first", "last", "number", NULL};

  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOi", kwlist, &first, &last,
                                   &self->number))
  {
    return -1;
  }

  if (first) {
    tmp = self->first;
    Py_INCREF(first);
    self->first = first;
    Py_XDECREF(tmp);
  }

  if (last) {
    tmp = self->last;
    Py_INCREF(last);
    self->last = last;
    Py_XDECREF(tmp);
  }

  return 0;
}

PyObject* Noddy4_name(Noddy4* self) {
  if (self->first == NULL) {
    PyErr_SetString(PyExc_AttributeError, "first");
    return NULL;
  }

  if (self->last == NULL) {
    PyErr_SetString(PyExc_AttributeError, "last");
    return NULL;
  }

  return PyUnicode_FromFormat("%S %S", self->first, self->last);
}
