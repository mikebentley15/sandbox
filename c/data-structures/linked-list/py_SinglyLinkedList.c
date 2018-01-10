#include <Python.h>
#include <structmember.h>

#include "SinglyLinkedList.h"

#include <stdbool.h>

typedef struct {
  PyObject_HEAD
  SLL* list;
} SLLWrap;

// For allowing 0 <= idx < size
static bool SLLWrap_update_idx(SLLWrap* self, int *idx) {
  if (*idx < 0) {
    *idx = self->list->size + *idx;
  }
  if (self->list->size <= (unsigned)*idx) {
    PyErr_SetString(PyExc_IndexError, "linked list index out of range");
    return false;
  }
  return true;
}

// For allowing 0 <= idx <= size
static bool SLLWrap_update_idx_extended(SLLWrap* self, int *idx) {
  if (*idx < 0) {
    *idx = self->list->size + *idx + 1;
  }
  if (self->list->size < (unsigned)*idx) {
    PyErr_SetString(PyExc_IndexError, "linked list index out of range");
    return false;
  }
  return true;
}

static PyObject* SLLWrap_clear(SLLWrap* self);

static int SLLWrap_traverse(SLLWrap* self, visitproc visit, void* arg) {
  for (SLLNode* current = self->list->head; current != NULL;
       current = current->next)
  {
    Py_VISIT((PyObject*)current->value);
  }
  return 0;
}

static int SLLWrap_clear_refs(SLLWrap* self) {
  for (SLLNode* current = self->list->head; current != NULL;
       current = current->next)
  {
    Py_CLEAR(current->value);
  }
  return 0;
}

static void SLLWrap_dealloc(SLLWrap* self) {
  PyObject_GC_UnTrack(self);
  SLLWrap_clear(self);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* SLLWrap_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
  SLLWrap* self;
  self = (SLLWrap*)type->tp_alloc(type, 0);
  if (self != NULL) {
    self->list = SLL_create();
    if (self->list == NULL) {
      Py_DECREF(self);
      return NULL;
    }
  }

  return (PyObject*)self;
}

static int SLLWrap_init(SLLWrap* self, PyObject* args, PyObject* kwds) {
  // init(self, iterable)
  // TODO: Be able to handle an input iterable.  For now, just be empty.
  return 0;
}

static PyObject* SLLWrap_getsize(SLLWrap* self, void* closure) {
  return PyLong_FromLong(self->list->size);
}

static PyObject* SLLWrap_str(SLLWrap* self) {
  if (0 == self->list->size) {
    return PyUnicode_FromString("[]");
  }

  PyObject* new_list = PyList_New(self->list->size);
  SLLNode* current = self->list->head;
  for (unsigned i = 0; i < self->list->size; i++) {
    PyObject* new_string = PyObject_Str((PyObject*)current->value);
    if (i == 0) {
      PyObject* tmp = new_string;
      PyObject* start = PyUnicode_FromOrdinal('[');
      new_string = PyUnicode_Concat(start, tmp);
      Py_DECREF(tmp);
      Py_DECREF(start);
    }
    if (current->next == NULL) {
      PyObject* tmp = new_string;
      PyObject* stop  = PyUnicode_FromOrdinal(']');
      new_string = PyUnicode_Concat(tmp, stop);
      Py_DECREF(tmp);
      Py_DECREF(stop);
    }
    PyList_SET_ITEM(new_list, i, new_string);
    current = current->next;
  }

  PyObject* joiner = PyUnicode_FromString(" -> ");
  PyObject* return_string = PyUnicode_Join(joiner, new_list);
  Py_DECREF(joiner);
  Py_DECREF(new_list);

  // TODO: finish testing and implementing
  return return_string;
}

static PyObject* SLLWrap_insert(SLLWrap* self, PyObject *args) {
  int idx = -1;
  PyObject* value = NULL;
  if (!PyArg_ParseTuple(args, "iO", &idx, &value)) {
    return NULL;
  }
  if (!SLLWrap_update_idx_extended(self, &idx)) {
    return NULL;
  }
  SLL_insert(self->list, value, idx);
  Py_INCREF(value);
  Py_RETURN_NONE;
}

static PyObject* SLLWrap_push(SLLWrap* self, PyObject *args) {
  PyObject* value = NULL;
  if (!PyArg_ParseTuple(args, "O", &value)) {
    return NULL;
  }
  SLL_push(self->list, value);
  Py_RETURN_NONE;
}

static PyObject* SLLWrap_remove(SLLWrap* self, PyObject *args) {
  int idx = -1;
  if (!PyArg_ParseTuple(args, "i", &idx)) {
    return NULL;
  }
  if (!SLLWrap_update_idx(self, &idx)) {
    return NULL;
  }
  PyObject* removed = (PyObject*)SLL_remove(self->list, idx);
  // Give ownership of this reference to the caller
  return removed;
}

static PyObject* SLLWrap_pop(SLLWrap* self) {
  if (self->list->size == 0) {
    PyErr_SetString(PyExc_IndexError, "pop from empty linked list");
    return NULL;
  }
  PyObject* removed = (PyObject*)SLL_pop(self->list);
  // Give ownership of this reference to the caller
  return removed;
}

static PyObject* SLLWrap_clear(SLLWrap* self) {
  SLLWrap_clear_refs(self);
  SLL_clear(self->list);
  Py_RETURN_NONE;
}

static PyObject* SLLWrap_at(SLLWrap* self, PyObject *args) {
  int idx;
  if (!PyArg_ParseTuple(args, "i", &idx)) {
    return NULL;
  }
  if (!SLLWrap_update_idx(self, &idx)) {
    return NULL;
  }
  PyObject* value = SLL_at(self->list, idx);
  Py_XINCREF(value);
  return value;
}


static PyMemberDef SLLWrap_members[] = {{NULL}}; // Sentinel
static PyGetSetDef SLLWrap_getsetters[] = {
  {"size", (getter)SLLWrap_getsize, (setter)NULL, "list size", NULL},
  { NULL }  // Sentinel
};
static PyMethodDef SLLWrap_methods[] = {
  {"insert", (PyCFunction)SLLWrap_insert, METH_VARARGS,
   "insert(idx, value): Insert value into the list at idx"},
  {"push", (PyCFunction)SLLWrap_push, METH_VARARGS,
   "push(value): Same as insert(0, value)"},
  {"remove", (PyCFunction)SLLWrap_remove, METH_VARARGS,
   "remove(idx): Remove and return the value at idx"},
  {"pop", (PyCFunction)SLLWrap_pop, METH_NOARGS,
   "pop(): Same as remove(0)"},
  {"clear", (PyCFunction)SLLWrap_clear, METH_NOARGS,
   "clear(): Remove all elements from the list"},
  {"at", (PyCFunction)SLLWrap_at, METH_VARARGS,
   "at(idx): Return the value at idx"},
  { NULL }  // Sentinel
};

static PyTypeObject SLLWrapType = {
  PyVarObject_HEAD_INIT(NULL, 0)
  "cdatastruct.SinglyLinkedList",   /* tp_name */
  sizeof(SLLWrap),             /* tp_basicsize */
  0,                         /* tp_itemsize */
  (destructor)SLLWrap_dealloc, /* tp_dealloc */
  0,                         /* tp_print */
  0,                         /* tp_getattr */
  0,                         /* tp_setattr */
  0,                         /* tp_reserved */
  (reprfunc)SLLWrap_str,       /* tp_repr */
  0,                         /* tp_as_number */
  0,                         /* tp_as_sequence */
  0,                         /* tp_as_mapping */
  0,                         /* tp_hash  */
  0,                         /* tp_call */
  (reprfunc)SLLWrap_str,       /* tp_str */
  0,                         /* tp_getattro */
  0,                         /* tp_setattro */
  0,                         /* tp_as_buffer */
  Py_TPFLAGS_DEFAULT |
    Py_TPFLAGS_BASETYPE |
    Py_TPFLAGS_HAVE_GC,      /* tp_flags */
  "C implementation of singly linked list",   /* tp_doc */
  (traverseproc)SLLWrap_traverse, /* tp_traverse */
  (inquiry)SLLWrap_clear_refs, /* tp_clear */
  0,                         /* tp_richcompare */
  0,                         /* tp_weaklistoffset */
  0,                         /* tp_iter */
  0,                         /* tp_iternext */
  SLLWrap_methods,             /* tp_methods */
  SLLWrap_members,             /* tp_members */
  SLLWrap_getsetters,          /* tp_getset */
  0,                         /* tp_base */
  0,                         /* tp_dict */
  0,                         /* tp_descr_get */
  0,                         /* tp_descr_set */
  0,                         /* tp_dictoffset */
  (initproc)SLLWrap_init,      /* tp_init */
  0,                         /* tp_alloc */
  SLLWrap_new,                 /* tp_new */
};

static PyMethodDef cdatastruct_functions[] = {
  { NULL } // Sentinel
};

static struct PyModuleDef cdatastruct_module = {
  PyModuleDef_HEAD_INIT,
  "cdatastruct",  // name of module
  // module documentation
  "C implementation of data structures",
  -1,             // Size of per-interpreter state of the module
                  // or -1 if the module keeps state in global variables.
  cdatastruct_functions
};

PyMODINIT_FUNC PyInit_cdatastruct(void) {
  PyObject *m;

  // Populate defaults into empty slots of all class types
  if (PyType_Ready(&SLLWrapType) < 0) return NULL;

  // Create the module
  m = PyModule_Create(&cdatastruct_module);
  if (m == NULL) return NULL;

  // Add objects to the module
  Py_INCREF(&SLLWrapType);
  PyModule_AddObject(m, "SinglyLinkedList", (PyObject*)&SLLWrapType);

  // Return the module
  return m;
}
