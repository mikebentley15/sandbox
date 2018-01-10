import sys

from PyQt5.QtCore import pyqtProperty, QObject

class Person(QObject):
    '''
    This is the type that will be registered with QML.  It must be a sub-class
    of QObject.
    '''
    def __init__(self, parent=None):
        'Initializes the value of the properties'
        super().__init__(parent)
        self._name = ''
        self._shoe_size = 0
    
    @pyqtProperty('QString')
    def name(self):
        '''
        Define the getter of the 'name' property.  The C++ type of the property
        is QString which Python will convert to and from a string.
        '''
        return self._name
    
    @name.setter
    def name(self, name):
        '''Define the setter of the 'name' property'''
        self._name = name
    
    @pyqtProperty(int)
    def shoe_size(self):
        '''
        Define the getter of the 'shoe_size' property.  The C++ type and Python
        type of the property is int.
        '''
        return self._shoe_size

    @shoe_size.setter
    def shoe_size(self, shoe_size):
        '''Define the setter of the 'shoe_size' property'''
        self._shoe_size = shoe_size
