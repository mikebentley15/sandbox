#ifndef NODDY_H
#define NODDY_H

#include <Python.h>
#include <structmember.h>

//
// Noddy1: a simple object with nothing
//

typedef struct {
  PyObject_HEAD
  // Type-specific fields go here
} Noddy1;

extern PyTypeObject Noddy1Type;

//
// Noddy2: Members (first, last, number)
//

typedef struct {
  PyObject_HEAD
  PyObject *first;  // first name
  PyObject *last;   // last name
  int number;
} Noddy2;

extern PyMemberDef Noddy2_members[];
extern PyMethodDef Noddy2_methods[];
extern PyTypeObject Noddy2Type;

void Noddy2_dealloc(Noddy2* self);
PyObject* Noddy2_new(PyTypeObject *type, PyObject *args, PyObject *kwds);
int Noddy2_init(Noddy2 *self, PyObject *args, PyObject *kwds);
PyObject* Noddy2_name(Noddy2* self);

//
// Noddy3: getter + setter, restricting first and last to string
//

typedef struct {
  PyObject_HEAD
  PyObject *first;
  PyObject *last;
  int number;
} Noddy3;

extern PyMemberDef Noddy3_members[];
extern PyGetSetDef Noddy3_getsetters[];
extern PyMethodDef Noddy3_methods[];
extern PyTypeObject Noddy3Type;

void Noddy3_dealloc(Noddy3* self);
PyObject* Noddy3_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
int Noddy3_init(Noddy3* self, PyObject* args, PyObject* kwds);
PyObject* Noddy3_getfirst(Noddy3* self, void* closure);
int Noddy3_setfirst(Noddy3* self, PyObject* value, void* closure);
PyObject* Noddy3_getlast(Noddy3* self, void* closure);
int Noddy3_setlast(Noddy3* self, PyObject* value, void* closure);
PyObject* Noddy3_name(Noddy3* self);

//
// Noddy4: Support cyclic garbage collection
//

typedef struct {
  PyObject_HEAD
  PyObject *first;
  PyObject *last;
  int number;
} Noddy4;

extern PyMemberDef Noddy4_members[];
extern PyMethodDef Noddy4_methods[];
extern PyTypeObject Noddy4Type;

int Noddy4_traverse(Noddy4* self, visitproc visit, void* arg);
int Noddy4_clear(Noddy4* self);
void Noddy4_dealloc(Noddy4* self);
PyObject* Noddy4_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
int Noddy4_init(Noddy4* self, PyObject* args, PyObject* kwds);
PyObject* Noddy4_name(Noddy4* self);


#endif // NODDY_H
