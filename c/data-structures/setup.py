from distutils.core import setup, Extension

cdatastruct_module = Extension(
        'cdatastruct',
        sources=[
            'linked-list/py_SinglyLinkedList.c',
            'linked-list/SinglyLinkedList.c',
            ])

setup(name='cdatastruct',
      version='1.0',
      description='Custom data structures in C',
      ext_modules=[cdatastruct_module])
