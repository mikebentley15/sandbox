#!/usr/bin/env python

import sys

from PyQt5.QtCore import QCoreApplication, QUrl
from PyQt5.QtQml import qmlRegisterType, QQmlComponent, QQmlEngine

from Person import Person

def main(arguments):
    'Main logic'
    app = QCoreApplication([__file__] + arguments)
    
    # Register the Python type.  Its URI is 'People', it's v1.0, and the type
    # will be called 'Person' in QML.
    qmlRegisterType(Person, 'People', 1, 0, 'Person')

    # Now we can load Person.qml, since it uses our registered python class
    engine = QQmlEngine()
    component = QQmlComponent(engine)
    component.loadUrl(QUrl('Person.qml'))
    person = component.create()
    
    if person is not None:
        print("The person's name is {0}.".format(person.name))
        print('They wear a size {0} shoe.'.format(person.shoe_size))
    else:
        for error in component.errors():
            print(error.toString())

if __name__ == '__main__':
    main(sys.argv[1:])
